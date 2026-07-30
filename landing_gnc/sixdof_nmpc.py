"""Reduced-order nonlinear MPC guidance for the 6-DOF landing truth plant.

The optimizer uses direct shooting over inertial position, velocity,
thrust-axis direction, closed-loop tilt rate, and mass. The 14-state truth
plant remains outside the optimizer. This separation is intentional: the
guidance model captures the dominant thrust-projection, attitude-bandwidth,
drag, and propellant couplings while the quaternion rigid body and engine
allocator provide the nonlinear rollout used for verification.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from .sixdof_control import guidance_acceleration
from .sixdof_math import normalize, quaternion_to_matrix
from .sixdof_models import (
    SixDofEnvironment,
    SixDofGuidance,
    SixDofNmpcConfig,
    SixDofState,
    SixDofVehicle,
)


@dataclass(frozen=True)
class NmpcGuidanceResult:
    desired_acceleration_inertial_mps2: np.ndarray
    diagnostics: dict[str, float]


@dataclass(frozen=True)
class PredictionRollout:
    positions_m: np.ndarray
    velocities_mps: np.ndarray
    thrust_axes: np.ndarray
    tilt_rates_radps: np.ndarray
    masses_kg: np.ndarray
    accelerations_mps2: np.ndarray
    thrust_accelerations_mps2: np.ndarray
    residual: np.ndarray
    objective: float
    maximum_corridor_violation_m: float
    maximum_tilt_deg: float
    minimum_propellant_kg: float
    valid: bool


class SixDofNmpcController:
    """Trust-region Gauss-Newton NMPC with nonlinear rollout acceptance."""

    def __init__(self, config: SixDofNmpcConfig | None = None) -> None:
        self.config = config or SixDofNmpcConfig()
        self.next_replan_time_s = 0.0
        self.plan_start_time_s = 0.0
        self.node_dt_s = 1.0
        self.plan: np.ndarray | None = None
        self.latest_diagnostics = self._empty_diagnostics()
        self.solve_count = 0
        self.accepted_solve_count = 0
        self.fallback_count = 0
        self.total_solve_time_ms = 0.0
        self.maximum_solve_time_ms = 0.0
        self.engine_out_contingency_event_count = 0
        self.in_engine_out_contingency = False

    def command(
        self,
        state: SixDofState,
        vehicle: SixDofVehicle,
        environment: SixDofEnvironment,
        guidance: SixDofGuidance,
        target_inertial_m: np.ndarray,
        active_engine_count: int = 4,
    ) -> NmpcGuidanceResult:
        if active_engine_count < 4:
            if not self.in_engine_out_contingency:
                self.engine_out_contingency_event_count += 1
            self.in_engine_out_contingency = True
            self.plan = None
            acceleration = guidance_acceleration(
                state,
                environment,
                guidance,
                target_inertial_m,
            )
            diagnostics = {
                **self.latest_diagnostics,
                "nmpc_replanned": 0.0,
                "nmpc_terminal_handoff": 0.0,
                "nmpc_fallback": 1.0,
                "nmpc_engine_out_contingency": 1.0,
                "nmpc_active_engines": float(active_engine_count),
            }
            return NmpcGuidanceResult(acceleration, diagnostics)
        self.in_engine_out_contingency = False

        if state.position_inertial_m[2] <= self.config.terminal_handoff_altitude_m:
            acceleration = guidance_acceleration(
                state,
                environment,
                guidance,
                target_inertial_m,
            )
            diagnostics = {
                **self.latest_diagnostics,
                "nmpc_replanned": 0.0,
                "nmpc_terminal_handoff": 1.0,
                "nmpc_fallback": 0.0,
                "nmpc_engine_out_contingency": 0.0,
            }
            return NmpcGuidanceResult(acceleration, diagnostics)

        replanned = state.time_s + 1.0e-9 >= self.next_replan_time_s
        if replanned:
            self.solve_count += 1
            started = time.perf_counter()
            plan, diagnostics = self._solve(
                state,
                vehicle,
                environment,
                guidance,
                np.asarray(target_inertial_m, dtype=float),
                active_engine_count,
            )
            solve_time_ms = 1.0e3 * (time.perf_counter() - started)
            self.total_solve_time_ms += solve_time_ms
            self.maximum_solve_time_ms = max(
                self.maximum_solve_time_ms,
                solve_time_ms,
            )
            accepted = bool(diagnostics["nmpc_nonlinear_rollout_valid"])
            if accepted:
                self.accepted_solve_count += 1
                self.plan = plan
                self.plan_start_time_s = state.time_s
                self.node_dt_s = diagnostics["nmpc_node_dt_s"]
            else:
                self.fallback_count += 1
                self.plan = None
            self.latest_diagnostics = {
                **diagnostics,
                "nmpc_solution_accepted": float(accepted),
                "nmpc_fallback": float(not accepted),
                "nmpc_solve_time_ms": solve_time_ms,
                "nmpc_active_engines": float(active_engine_count),
                "nmpc_engine_out_contingency": 0.0,
            }
            self.next_replan_time_s = state.time_s + self.config.replan_period_s

        diagnostics = {
            **self.latest_diagnostics,
            "nmpc_replanned": float(replanned),
            "nmpc_terminal_handoff": 0.0,
        }
        if self.plan is None or diagnostics["nmpc_fallback"] > 0.5:
            acceleration = guidance_acceleration(
                state,
                environment,
                guidance,
                target_inertial_m,
            )
            return NmpcGuidanceResult(acceleration, diagnostics)

        elapsed = max(0.0, state.time_s - self.plan_start_time_s)
        index = min(
            int(elapsed / max(self.node_dt_s, 1.0e-6)),
            len(self.plan) - 1,
        )
        thrust_acceleration = self.plan[index]
        acceleration = thrust_acceleration + np.array(
            [0.0, 0.0, -environment.gravity_mps2]
        )
        return NmpcGuidanceResult(acceleration, diagnostics)

    def summary(self) -> dict[str, float | int]:
        return {
            "nmpc_solve_count": self.solve_count,
            "nmpc_accepted_solve_count": self.accepted_solve_count,
            "nmpc_fallback_count": self.fallback_count,
            "nmpc_acceptance_rate": (
                self.accepted_solve_count / self.solve_count
                if self.solve_count
                else 0.0
            ),
            "nmpc_mean_solve_time_ms": (
                self.total_solve_time_ms / self.solve_count
                if self.solve_count
                else 0.0
            ),
            "nmpc_maximum_solve_time_ms": self.maximum_solve_time_ms,
            "nmpc_engine_out_contingency_event_count": (
                self.engine_out_contingency_event_count
            ),
        }

    def _solve(
        self,
        state: SixDofState,
        vehicle: SixDofVehicle,
        environment: SixDofEnvironment,
        guidance: SixDofGuidance,
        target: np.ndarray,
        active_engine_count: int,
    ) -> tuple[np.ndarray, dict[str, float]]:
        config = self.config
        horizon_s = choose_nmpc_horizon(state, config)
        node_dt_s = horizon_s / config.horizon_steps
        reference = cubic_trajectory_reference(
            state,
            target,
            horizon_s,
            config.horizon_steps,
            node_dt_s,
            config.terminal_descent_mps,
        )
        plan = self._initial_plan(
            state,
            vehicle,
            environment,
            guidance,
            target,
            reference,
            active_engine_count,
        )
        plan = project_thrust_plan(
            plan,
            state.mass_kg,
            vehicle,
            guidance,
            active_engine_count,
        )
        initial_rollout = nonlinear_prediction_rollout(
            state,
            plan,
            reference,
            vehicle,
            environment,
            guidance,
            config,
            target,
            node_dt_s,
            active_engine_count,
        )
        rollout = initial_rollout
        trust_radius = config.initial_trust_region_mps2
        accepted_steps = 0
        iteration_count = 0

        for iteration in range(config.maximum_iterations):
            iteration_count = iteration + 1
            jacobian = finite_difference_residual_jacobian(
                state,
                plan,
                rollout,
                reference,
                vehicle,
                environment,
                guidance,
                config,
                target,
                node_dt_s,
                active_engine_count,
            )
            normal_matrix = (
                jacobian.T @ jacobian
                + config.regularization * np.eye(plan.size)
            )
            gradient = jacobian.T @ rollout.residual
            try:
                step = -np.linalg.solve(normal_matrix, gradient)
            except np.linalg.LinAlgError:
                step = -np.linalg.lstsq(normal_matrix, gradient, rcond=None)[0]
            step = step.reshape(plan.shape)
            maximum_component = float(np.max(np.abs(step)))
            if maximum_component > trust_radius:
                step *= trust_radius / maximum_component

            improved = False
            for scale in (1.0, 0.5, 0.25):
                candidate = project_thrust_plan(
                    plan + scale * step,
                    state.mass_kg,
                    vehicle,
                    guidance,
                    active_engine_count,
                )
                candidate_rollout = nonlinear_prediction_rollout(
                    state,
                    candidate,
                    reference,
                    vehicle,
                    environment,
                    guidance,
                    config,
                    target,
                    node_dt_s,
                    active_engine_count,
                )
                required_drop = (
                    config.acceptance_relative_improvement
                    * max(1.0, rollout.objective)
                )
                if candidate_rollout.objective < rollout.objective - required_drop:
                    plan = candidate
                    rollout = candidate_rollout
                    accepted_steps += 1
                    trust_radius = min(
                        config.maximum_trust_region_mps2,
                        1.35 * trust_radius,
                    )
                    improved = True
                    break
            if not improved:
                trust_radius *= 0.5
                if trust_radius < config.minimum_trust_region_mps2:
                    break

        diagnostics = {
            "nmpc_replanned": 1.0,
            "nmpc_terminal_handoff": 0.0,
            "nmpc_solution_accepted": 0.0,
            "nmpc_fallback": 0.0,
            "nmpc_iterations": float(iteration_count),
            "nmpc_accepted_steps": float(accepted_steps),
            "nmpc_initial_objective": initial_rollout.objective,
            "nmpc_final_objective": rollout.objective,
            "nmpc_objective_reduction": (
                initial_rollout.objective - rollout.objective
            ),
            "nmpc_horizon_s": horizon_s,
            "nmpc_node_dt_s": node_dt_s,
            "nmpc_trust_region_mps2": trust_radius,
            "nmpc_predicted_terminal_position_error_m": float(
                np.linalg.norm(rollout.positions_m[-1] - target)
            ),
            "nmpc_predicted_terminal_speed_mps": float(
                np.linalg.norm(
                    rollout.velocities_mps[-1]
                    - np.array([0.0, 0.0, -config.terminal_descent_mps])
                )
            ),
            "nmpc_predicted_maximum_corridor_violation_m": (
                rollout.maximum_corridor_violation_m
            ),
            "nmpc_predicted_maximum_tilt_deg": rollout.maximum_tilt_deg,
            "nmpc_predicted_minimum_propellant_kg": (
                rollout.minimum_propellant_kg
            ),
            "nmpc_nonlinear_rollout_valid": float(rollout.valid),
            "nmpc_solve_time_ms": 0.0,
            "nmpc_active_engines": float(active_engine_count),
        }
        return plan, diagnostics

    def _initial_plan(
        self,
        state: SixDofState,
        vehicle: SixDofVehicle,
        environment: SixDofEnvironment,
        guidance: SixDofGuidance,
        target: np.ndarray,
        reference: dict[str, np.ndarray],
        active_engine_count: int,
    ) -> np.ndarray:
        if self.plan is not None and len(self.plan) == self.config.horizon_steps:
            elapsed = max(0.0, state.time_s - self.plan_start_time_s)
            shift_nodes = min(
                int(elapsed / max(self.node_dt_s, 1.0e-6)),
                len(self.plan) - 1,
            )
            if shift_nodes > 0:
                return np.vstack(
                    (
                        self.plan[shift_nodes:],
                        np.repeat(
                            self.plan[-1][None, :],
                            shift_nodes,
                            axis=0,
                        ),
                    )
                )
            return self.plan.copy()
        gravity = np.array([0.0, 0.0, -environment.gravity_mps2])
        plan = reference["accelerations_mps2"] - gravity
        baseline_acceleration = guidance_acceleration(
            state,
            environment,
            guidance,
            target,
        )
        plan[0] = 0.65 * plan[0] + 0.35 * (
            baseline_acceleration - gravity
        )
        return project_thrust_plan(
            plan,
            state.mass_kg,
            vehicle,
            guidance,
            active_engine_count,
        )

    @staticmethod
    def _empty_diagnostics() -> dict[str, float]:
        return {
            "nmpc_replanned": 0.0,
            "nmpc_terminal_handoff": 0.0,
            "nmpc_solution_accepted": 0.0,
            "nmpc_fallback": 1.0,
            "nmpc_iterations": 0.0,
            "nmpc_accepted_steps": 0.0,
            "nmpc_initial_objective": 0.0,
            "nmpc_final_objective": 0.0,
            "nmpc_objective_reduction": 0.0,
            "nmpc_horizon_s": 0.0,
            "nmpc_node_dt_s": 0.0,
            "nmpc_trust_region_mps2": 0.0,
            "nmpc_predicted_terminal_position_error_m": 0.0,
            "nmpc_predicted_terminal_speed_mps": 0.0,
            "nmpc_predicted_maximum_corridor_violation_m": 0.0,
            "nmpc_predicted_maximum_tilt_deg": 0.0,
            "nmpc_predicted_minimum_propellant_kg": 0.0,
            "nmpc_nonlinear_rollout_valid": 0.0,
            "nmpc_solve_time_ms": 0.0,
            "nmpc_active_engines": 4.0,
            "nmpc_engine_out_contingency": 0.0,
        }


def choose_nmpc_horizon(
    state: SixDofState,
    config: SixDofNmpcConfig,
) -> float:
    altitude = max(0.0, state.position_inertial_m[2])
    closing_speed = max(
        1.0,
        -state.velocity_inertial_mps[2] - config.terminal_descent_mps,
    )
    kinematic_time = 2.0 * altitude / closing_speed
    return float(
        np.clip(
            kinematic_time,
            config.minimum_horizon_s,
            config.maximum_horizon_s,
        )
    )


def cubic_trajectory_reference(
    state: SixDofState,
    target: np.ndarray,
    horizon_s: float,
    steps: int,
    node_dt_s: float,
    terminal_descent_mps: float,
) -> dict[str, np.ndarray]:
    times = node_dt_s * np.arange(1, steps + 1, dtype=float)
    start_position = state.position_inertial_m
    start_velocity = state.velocity_inertial_mps
    terminal_velocity = np.array([0.0, 0.0, -terminal_descent_mps])
    delta = target - start_position
    c2 = (
        3.0 * delta
        - (2.0 * start_velocity + terminal_velocity) * horizon_s
    ) / horizon_s**2
    c3 = (
        -2.0 * delta
        + (start_velocity + terminal_velocity) * horizon_s
    ) / horizon_s**3
    positions = (
        start_position
        + times[:, None] * start_velocity
        + times[:, None] ** 2 * c2
        + times[:, None] ** 3 * c3
    )
    velocities = (
        start_velocity
        + 2.0 * times[:, None] * c2
        + 3.0 * times[:, None] ** 2 * c3
    )
    accelerations = 2.0 * c2 + 6.0 * times[:, None] * c3
    positions[:, 2] = np.maximum(positions[:, 2], 0.0)
    return {
        "times_s": times,
        "positions_m": positions,
        "velocities_mps": velocities,
        "accelerations_mps2": accelerations,
    }


def project_thrust_plan(
    plan: np.ndarray,
    initial_mass_kg: float,
    vehicle: SixDofVehicle,
    guidance: SixDofGuidance,
    active_engine_count: int,
) -> np.ndarray:
    projected = np.asarray(plan, dtype=float).copy()
    maximum_thrust = (
        vehicle.maximum_engine_thrust_n * max(1, active_engine_count)
    )
    maximum_specific_thrust = maximum_thrust / max(
        initial_mass_kg,
        vehicle.dry_mass_kg,
    )
    tilt_tangent = math.tan(guidance.maximum_tilt_rad)
    for index, thrust in enumerate(projected):
        axial = max(0.05, float(thrust[2]))
        lateral = thrust[:2]
        lateral_norm = float(np.linalg.norm(lateral))
        maximum_lateral = axial * tilt_tangent
        if lateral_norm > maximum_lateral and lateral_norm > 0.0:
            lateral *= maximum_lateral / lateral_norm
        thrust[:] = [lateral[0], lateral[1], axial]
        magnitude = float(np.linalg.norm(thrust))
        if magnitude > maximum_specific_thrust:
            thrust *= maximum_specific_thrust / magnitude
        projected[index] = thrust
    return projected


def nonlinear_prediction_rollout(
    state: SixDofState,
    plan: np.ndarray,
    reference: dict[str, np.ndarray],
    vehicle: SixDofVehicle,
    environment: SixDofEnvironment,
    guidance: SixDofGuidance,
    config: SixDofNmpcConfig,
    target: np.ndarray,
    node_dt_s: float,
    active_engine_count: int,
) -> PredictionRollout:
    steps = len(plan)
    positions = np.zeros((steps, 3))
    velocities = np.zeros((steps, 3))
    axes = np.zeros((steps, 3))
    rates = np.zeros((steps, 3))
    masses = np.zeros(steps)
    accelerations = np.zeros((steps, 3))
    position = state.position_inertial_m.copy()
    velocity = state.velocity_inertial_mps.copy()
    body_axis = quaternion_to_matrix(
        state.quaternion_body_to_inertial
    )[:, 2]
    angular_velocity_inertial = (
        quaternion_to_matrix(state.quaternion_body_to_inertial)
        @ state.angular_velocity_body_radps
    )
    tilt_rate = (
        angular_velocity_inertial
        - np.dot(angular_velocity_inertial, body_axis) * body_axis
    )
    mass = state.mass_kg
    gravity = np.array([0.0, 0.0, -environment.gravity_mps2])
    wind = np.asarray(environment.wind_inertial_mps, dtype=float)
    maximum_thrust = (
        vehicle.maximum_engine_thrust_n * max(1, active_engine_count)
    )

    for index, commanded_thrust_acceleration in enumerate(plan):
        commanded_axis = normalize(
            commanded_thrust_acceleration,
            np.array([0.0, 0.0, 1.0]),
        )
        attitude_substeps = max(1, int(math.ceil(node_dt_s / 0.12)))
        attitude_dt_s = node_dt_s / attitude_substeps
        for _ in range(attitude_substeps):
            axis_error = np.cross(body_axis, commanded_axis)
            rate_derivative = (
                config.attitude_natural_frequency_radps**2 * axis_error
                - 2.0
                * config.attitude_damping_ratio
                * config.attitude_natural_frequency_radps
                * tilt_rate
            )
            tilt_rate = tilt_rate + attitude_dt_s * rate_derivative
            body_axis = normalize(
                body_axis
                + attitude_dt_s * np.cross(tilt_rate, body_axis),
                np.array([0.0, 0.0, 1.0]),
            )
        requested_thrust = mass * float(
            np.linalg.norm(commanded_thrust_acceleration)
        )
        thrust = min(maximum_thrust, requested_thrust)
        relative_velocity = velocity - wind
        relative_speed = float(np.linalg.norm(relative_velocity))
        drag = (
            -0.5
            * environment.air_density_kg_m3
            * vehicle.drag_coefficient
            * vehicle.reference_area_m2
            * relative_speed
            * relative_velocity
        )
        acceleration = thrust / mass * body_axis + gravity + drag / mass
        position = (
            position
            + node_dt_s * velocity
            + 0.5 * node_dt_s**2 * acceleration
        )
        velocity = velocity + node_dt_s * acceleration
        mass = max(
            vehicle.dry_mass_kg,
            mass
            - node_dt_s
            * thrust
            / (vehicle.specific_impulse_s * 9.80665),
        )
        positions[index] = position
        velocities[index] = velocity
        axes[index] = body_axis
        rates[index] = tilt_rate
        masses[index] = mass
        accelerations[index] = acceleration

    residual = prediction_residual(
        positions,
        velocities,
        axes,
        rates,
        masses,
        plan,
        reference,
        target,
        vehicle,
        config,
    )
    horizontal_error = np.linalg.norm(positions[:, :2] - target[:2], axis=1)
    corridor_half_width = 0.65 + 0.020 * np.maximum(positions[:, 2], 0.0)
    corridor_violation = np.maximum(
        horizontal_error - corridor_half_width,
        0.0,
    )
    tilts = np.degrees(
        np.arccos(np.clip(axes[:, 2], -1.0, 1.0))
    )
    minimum_propellant = float(
        np.min(masses - vehicle.dry_mass_kg)
    )
    valid = bool(
        np.all(np.isfinite(residual))
        and np.all(positions[:, 2] >= -2.0)
        and minimum_propellant > 0.0
        and np.max(tilts) <= math.degrees(guidance.maximum_tilt_rad) + 3.0
    )
    return PredictionRollout(
        positions_m=positions,
        velocities_mps=velocities,
        thrust_axes=axes,
        tilt_rates_radps=rates,
        masses_kg=masses,
        accelerations_mps2=accelerations,
        thrust_accelerations_mps2=plan.copy(),
        residual=residual,
        objective=0.5 * float(np.dot(residual, residual)),
        maximum_corridor_violation_m=float(np.max(corridor_violation)),
        maximum_tilt_deg=float(np.max(tilts)),
        minimum_propellant_kg=minimum_propellant,
        valid=valid,
    )


def prediction_residual(
    positions: np.ndarray,
    velocities: np.ndarray,
    axes: np.ndarray,
    rates: np.ndarray,
    masses: np.ndarray,
    plan: np.ndarray,
    reference: dict[str, np.ndarray],
    target: np.ndarray,
    vehicle: SixDofVehicle,
    config: SixDofNmpcConfig,
) -> np.ndarray:
    progress = np.linspace(0.35, 1.0, len(plan))[:, None]
    residuals = [
        math.sqrt(config.position_weight)
        * progress
        * (positions - reference["positions_m"]),
        math.sqrt(config.velocity_weight)
        * progress
        * (velocities - reference["velocities_mps"]),
        math.sqrt(config.attitude_lag_weight)
        * np.cross(
            axes,
            plan / np.maximum(
                np.linalg.norm(plan, axis=1)[:, None],
                1.0e-9,
            ),
        ),
        math.sqrt(config.body_rate_weight) * rates,
        math.sqrt(config.thrust_weight) * plan,
    ]
    slew = np.vstack((np.zeros((1, 3)), np.diff(plan, axis=0)))
    residuals.append(math.sqrt(config.thrust_slew_weight) * slew)
    horizontal_error = np.linalg.norm(
        positions[:, :2] - target[:2],
        axis=1,
    )
    corridor = 0.65 + 0.020 * np.maximum(positions[:, 2], 0.0)
    residuals.append(
        math.sqrt(config.corridor_weight)
        * np.maximum(horizontal_error - corridor, 0.0)[:, None]
    )
    reserve_violation = np.maximum(
        150.0 - (masses - vehicle.dry_mass_kg),
        0.0,
    )
    residuals.append(
        math.sqrt(config.propellant_reserve_weight)
        * reserve_violation[:, None]
        / 150.0
    )
    terminal_position = (
        math.sqrt(config.terminal_position_weight)
        * (positions[-1] - target)
    )
    terminal_velocity = (
        math.sqrt(config.terminal_velocity_weight)
        * (
            velocities[-1]
            - np.array([0.0, 0.0, -config.terminal_descent_mps])
        )
    )
    residuals.extend(
        (terminal_position[None, :], terminal_velocity[None, :])
    )
    return np.concatenate(
        [residual.reshape(-1) for residual in residuals]
    )


def finite_difference_residual_jacobian(
    state: SixDofState,
    plan: np.ndarray,
    baseline: PredictionRollout,
    reference: dict[str, np.ndarray],
    vehicle: SixDofVehicle,
    environment: SixDofEnvironment,
    guidance: SixDofGuidance,
    config: SixDofNmpcConfig,
    target: np.ndarray,
    node_dt_s: float,
    active_engine_count: int,
) -> np.ndarray:
    jacobian = np.zeros((len(baseline.residual), plan.size))
    step = config.finite_difference_step_mps2
    for index in range(plan.size):
        perturbed = plan.copy().reshape(-1)
        perturbed[index] += step
        projected = project_thrust_plan(
            perturbed.reshape(plan.shape),
            state.mass_kg,
            vehicle,
            guidance,
            active_engine_count,
        )
        rollout = nonlinear_prediction_rollout(
            state,
            projected,
            reference,
            vehicle,
            environment,
            guidance,
            config,
            target,
            node_dt_s,
            active_engine_count,
        )
        effective_step = projected.reshape(-1)[index] - plan.reshape(-1)[index]
        if abs(effective_step) > 1.0e-8:
            jacobian[:, index] = (
                rollout.residual - baseline.residual
            ) / effective_step
    return jacobian
