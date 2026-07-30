import math
import unittest

import numpy as np

from landing_gnc.sixdof_control import (
    allocate_engine_commands,
    wrench_command,
)
from landing_gnc.sixdof_math import (
    matrix_to_quaternion,
    quaternion_from_euler,
    quaternion_to_euler,
    quaternion_to_matrix,
)
from landing_gnc.sixdof_models import (
    SixDofAttitudeControl,
    SixDofEnvironment,
    SixDofGuidance,
    SixDofNmpcConfig,
    SixDofScenario,
    SixDofVehicle,
)
from landing_gnc.sixdof_nmpc import (
    SixDofNmpcController,
    cubic_trajectory_reference,
    project_thrust_plan,
)
from landing_gnc.sixdof_sim import (
    default_sixdof_initial_state,
    run_sixdof_simulation,
)


class SixDofMathTests(unittest.TestCase):
    def test_quaternion_rotation_round_trip(self):
        expected_euler = np.radians([8.0, -5.0, 17.0])
        quaternion = quaternion_from_euler(*expected_euler)
        rotation = quaternion_to_matrix(quaternion)
        reconstructed = matrix_to_quaternion(rotation)

        self.assertAlmostEqual(
            abs(float(np.dot(quaternion, reconstructed))),
            1.0,
            places=12,
        )
        np.testing.assert_allclose(
            quaternion_to_euler(reconstructed),
            expected_euler,
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            rotation.T @ rotation,
            np.eye(3),
            atol=1.0e-12,
        )

    def test_propellant_depletion_reduces_inertia_and_moves_cm_arm(self):
        vehicle = SixDofVehicle()
        wet_inertia = vehicle.inertia_diag_kg_m2(
            vehicle.wet_mass_kg
        )
        dry_inertia = vehicle.inertia_diag_kg_m2(
            vehicle.dry_mass_kg
        )
        inertia_rate = vehicle.inertia_rate_diag_kg_m2_s(-80.0)
        wet_arm = abs(
            vehicle.engine_positions_body_m(
                vehicle.wet_mass_kg
            )[0][2]
        )
        dry_arm = abs(
            vehicle.engine_positions_body_m(
                vehicle.dry_mass_kg
            )[0][2]
        )

        self.assertTrue(np.all(dry_inertia < wet_inertia))
        self.assertTrue(np.all(inertia_rate < 0.0))
        self.assertGreater(dry_arm, wet_arm)


class SixDofControlTests(unittest.TestCase):
    def test_four_engine_allocator_reconstructs_nominal_wrench(self):
        vehicle = SixDofVehicle()
        environment = SixDofEnvironment()
        state = default_sixdof_initial_state(vehicle)
        requested = wrench_command(
            state,
            vehicle,
            environment,
            SixDofGuidance(),
            SixDofAttitudeControl(),
            np.zeros(3),
        )
        allocated = allocate_engine_commands(
            state,
            requested,
            vehicle,
        )

        self.assertEqual(len(allocated.engine_commands), 4)
        self.assertLess(
            allocated.normalized_allocation_residual,
            0.01,
        )
        self.assertGreater(
            abs(allocated.torque_body_nm[2]),
            1.0e3,
        )
        for command in allocated.engine_commands:
            self.assertLessEqual(
                math.hypot(
                    command.gimbal_x_rad,
                    command.gimbal_y_rad,
                ),
                vehicle.maximum_gimbal_rad + 1.0e-12,
            )
            self.assertGreaterEqual(command.throttle, 0.0)
            self.assertLessEqual(command.throttle, 1.0)


class SixDofNmpcTests(unittest.TestCase):
    def test_cubic_reference_satisfies_terminal_boundary_conditions(self):
        vehicle = SixDofVehicle()
        state = default_sixdof_initial_state(vehicle)
        target = np.zeros(3)
        reference = cubic_trajectory_reference(
            state,
            target,
            horizon_s=24.0,
            steps=8,
            node_dt_s=3.0,
            terminal_descent_mps=1.0,
        )

        np.testing.assert_allclose(
            reference["positions_m"][-1],
            target,
            atol=1.0e-10,
        )
        np.testing.assert_allclose(
            reference["velocities_mps"][-1],
            np.array([0.0, 0.0, -1.0]),
            atol=1.0e-10,
        )

    def test_thrust_projection_enforces_tilt_and_engine_out_authority(self):
        vehicle = SixDofVehicle()
        guidance = SixDofGuidance()
        plan = np.tile(np.array([20.0, -15.0, 4.0]), (8, 1))
        projected = project_thrust_plan(
            plan,
            vehicle.wet_mass_kg,
            vehicle,
            guidance,
            active_engine_count=3,
        )
        maximum_specific_thrust = (
            3.0
            * vehicle.maximum_engine_thrust_n
            / vehicle.wet_mass_kg
        )

        self.assertTrue(
            np.all(
                np.linalg.norm(projected, axis=1)
                <= maximum_specific_thrust + 1.0e-12
            )
        )
        lateral = np.linalg.norm(projected[:, :2], axis=1)
        self.assertTrue(
            np.all(
                lateral
                <= projected[:, 2]
                * math.tan(guidance.maximum_tilt_rad)
                + 1.0e-12
            )
        )

    def test_nmpc_accepts_nonlinear_rollout_from_initial_condition(self):
        vehicle = SixDofVehicle()
        state = default_sixdof_initial_state(vehicle)
        controller = SixDofNmpcController(SixDofNmpcConfig())
        result = controller.command(
            state,
            vehicle,
            SixDofEnvironment(
                wind_inertial_mps=(12.0, -6.0, 0.0)
            ),
            SixDofGuidance(),
            np.zeros(3),
        )

        self.assertEqual(
            result.diagnostics["nmpc_solution_accepted"],
            1.0,
        )
        self.assertEqual(
            result.diagnostics["nmpc_nonlinear_rollout_valid"],
            1.0,
        )
        self.assertGreater(
            result.diagnostics["nmpc_objective_reduction"],
            0.0,
        )

    def test_engine_out_invalidates_nominal_nmpc_plan(self):
        vehicle = SixDofVehicle()
        controller = SixDofNmpcController(SixDofNmpcConfig())
        result = controller.command(
            default_sixdof_initial_state(vehicle),
            vehicle,
            SixDofEnvironment(),
            SixDofGuidance(),
            np.zeros(3),
            active_engine_count=3,
        )

        self.assertEqual(
            result.diagnostics["nmpc_engine_out_contingency"],
            1.0,
        )
        self.assertEqual(result.diagnostics["nmpc_fallback"], 1.0)
        self.assertEqual(
            controller.summary()[
                "nmpc_engine_out_contingency_event_count"
            ],
            1,
        )


class SixDofClosedLoopTests(unittest.TestCase):
    def test_nominal_and_crosswind_cases_land(self):
        cases = (
            SixDofEnvironment(
                wind_inertial_mps=(0.0, 0.0, 0.0)
            ),
            SixDofEnvironment(
                wind_inertial_mps=(12.0, -6.0, 0.0)
            ),
        )
        for environment in cases:
            with self.subTest(wind=environment.wind_inertial_mps):
                rows, metrics, _ = run_sixdof_simulation(
                    duration_s=55.0,
                    dt_s=0.05,
                    environment=environment,
                )
                self.assertTrue(metrics["success"])
                self.assertLess(
                    metrics["maximum_quaternion_norm_error"],
                    1.0e-12,
                )
                self.assertEqual(rows[-1]["z_m"], 0.0)
                self.assertLess(
                    metrics["p95_allocation_residual"],
                    0.01,
                )

    def test_high_wind_identifies_footprint_boundary(self):
        _, metrics, _ = run_sixdof_simulation(
            duration_s=55.0,
            dt_s=0.05,
            environment=SixDofEnvironment(
                wind_inertial_mps=(18.0, -10.0, 0.0)
            ),
        )

        self.assertFalse(metrics["success"])
        self.assertGreater(
            metrics["horizontal_target_error_m"],
            3.0,
        )
        self.assertLess(metrics["maximum_tilt_deg"], 12.0)
        self.assertGreater(
            metrics["propellant_remaining_kg"],
            0.0,
        )

    def test_mid_descent_engine_out_reduces_wrench_authority(self):
        rows, metrics, _ = run_sixdof_simulation(
            duration_s=55.0,
            dt_s=0.05,
            scenario=SixDofScenario(
                name="engine_out",
                failed_engine_index=0,
                engine_failure_time_s=12.0,
            ),
        )

        self.assertFalse(metrics["success"])
        self.assertGreater(
            metrics["horizontal_target_error_m"],
            3.0,
        )
        self.assertGreater(
            metrics["p95_allocation_residual"],
            0.1,
        )
        self.assertTrue(
            any(
                row["time_s"] > 12.1
                and row["engine_1_enabled"] == 0
                and row["engine_1_throttle"] == 0.0
                for row in rows
            )
        )

    def test_nmpc_reduces_high_crosswind_miss_distance(self):
        environment = SixDofEnvironment(
            wind_inertial_mps=(18.0, -10.0, 0.0)
        )
        _, baseline, _ = run_sixdof_simulation(
            duration_s=55.0,
            dt_s=0.05,
            environment=environment,
            guidance_mode="baseline",
        )
        rows, nmpc, _ = run_sixdof_simulation(
            duration_s=55.0,
            dt_s=0.05,
            environment=environment,
            guidance_mode="nmpc",
        )

        self.assertLess(
            nmpc["horizontal_target_error_m"],
            baseline["horizontal_target_error_m"],
        )
        self.assertGreater(nmpc["nmpc_acceptance_rate"], 0.95)
        self.assertTrue(
            any(row.get("nmpc_terminal_handoff", 0.0) for row in rows)
        )


if __name__ == "__main__":
    unittest.main()
