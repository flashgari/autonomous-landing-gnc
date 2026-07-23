import unittest

import numpy as np

from landing_gnc.constrained_guidance import (
    PredictiveGuidanceController,
    prediction_matrices,
)
from landing_gnc.models import (
    AttitudeControl,
    Environment,
    Guidance,
    PredictiveGuidanceConfig,
    State,
    Vehicle,
)
from landing_gnc.sim import run_simulation


class ConstrainedGuidanceTests(unittest.TestCase):
    def test_prediction_matrices_match_constant_acceleration_kinematics(self):
        matrices = prediction_matrices(4, 0.5)
        acceleration = np.full(4, 2.0)
        predicted_velocity_change = matrices["velocity"] @ acceleration
        predicted_position_change = matrices["position"] @ acceleration
        self.assertAlmostEqual(predicted_velocity_change[-1], 4.0)
        self.assertAlmostEqual(predicted_position_change[-1], 4.0)

    def test_nominal_qp_returns_a_feasible_accepted_command(self):
        vehicle = Vehicle()
        state = State(
            time_s=0.0,
            x_m=18.0,
            z_m=720.0,
            vx_mps=-2.5,
            vz_mps=-58.0,
            theta_rad=0.0,
            omega_radps=0.0,
            mass_kg=vehicle.wet_mass_kg,
        )
        controller = PredictiveGuidanceController(PredictiveGuidanceConfig())
        result = controller.command(
            state,
            vehicle,
            Environment(),
            Guidance(),
            AttitudeControl(),
        )
        self.assertEqual(result.diagnostics["optimizer_solution_accepted"], 1.0)
        self.assertLess(
            result.diagnostics["optimizer_max_constraint_violation"],
            0.01,
        )
        self.assertLess(abs(result.command.desired_theta_rad), 0.14)

    def test_predictive_guidance_lands_with_ekf_and_flight_like_actuators(self):
        rows, metrics, _ = run_simulation(
            duration_s=70.0,
            dt_s=0.05,
            guidance_mode="predictive",
            navigation_mode="ekf",
            actuator_mode="flight_like",
            rng_seed=4242,
        )
        self.assertTrue(metrics["success"])
        self.assertLess(abs(metrics["landing_x_error_m"]), 1.0)
        self.assertGreater(metrics["optimizer_acceptance_rate"], 0.90)
        self.assertTrue(any(row["optimizer_terminal_handoff"] for row in rows))


if __name__ == "__main__":
    unittest.main()
