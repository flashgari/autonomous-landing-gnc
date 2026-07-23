"""Condensed direct-transcription guidance solved as a constrained convex QP.

The optimizer uses a double-integrator prediction model in downrange and
altitude. Position and velocity states are analytically condensed into the
future acceleration sequence. An in-repository ADMM solver handles the small
quadratic program without hiding feasibility residuals behind a solver API.
"""

import math
from dataclasses import dataclass

import numpy as np

from .dynamics import clamp
from .guidance import (
    command_from_acceleration,
    corridor_guidance_command,
    vertical_velocity_reference,
)
from .models import (
    AttitudeControl,
    Command,
    Environment,
    Guidance,
    PredictiveGuidanceConfig,
    State,
    Vehicle,
)


@dataclass(frozen=True)
class PredictiveGuidanceResult:
    command: Command
    diagnostics: dict[str, float]


@dataclass(frozen=True)
class QpSolution:
    decision: np.ndarray
    iterations: int
    converged: bool
    primal_residual: float
    dual_residual: float
    maximum_violation: float
    objective: float


class PredictiveGuidanceController:
    """Receding-horizon constrained acceleration guidance."""

    def __init__(self, config: PredictiveGuidanceConfig | None = None) -> None:
        self.config = config or PredictiveGuidanceConfig()
        self.next_replan_time_s = 0.0
        self.planned_ax_mps2 = 0.0
        self.planned_az_mps2 = 0.0
        self.previous_ax_mps2 = 0.0
        self.previous_az_mps2 = 0.0
        self.latest_diagnostics = self._empty_diagnostics()
        self.warm_decision: np.ndarray | None = None
        self.solve_count = 0
        self.converged_solve_count = 0
        self.accepted_solve_count = 0
        self.fallback_count = 0
        self.iteration_sum = 0
        self.maximum_observed_violation = 0.0

    def command(
        self,
        state: State,
        vehicle: Vehicle,
        environment: Environment,
        guidance: Guidance,
        attitude: AttitudeControl,
        target_x_m: float = 0.0,
    ) -> PredictiveGuidanceResult:
        if state.z_m <= self.config.terminal_handoff_altitude_m:
            terminal_command = corridor_guidance_command(
                state,
                vehicle,
                environment,
                guidance,
                attitude,
                target_x_m,
            )
            diagnostics = {
                **self.latest_diagnostics,
                "optimizer_replanned": 0.0,
                "optimizer_terminal_handoff": 1.0,
            }
            return PredictiveGuidanceResult(terminal_command, diagnostics)

        replanned = state.time_s + 1.0e-9 >= self.next_replan_time_s
        if replanned:
            solution, plan_diagnostics = self._solve_plan(
                state,
                vehicle,
                environment,
                guidance,
                target_x_m,
            )
            self.solve_count += 1
            self.iteration_sum += solution.iterations
            self.maximum_observed_violation = max(
                self.maximum_observed_violation,
                solution.maximum_violation,
            )
            accepted = (
                solution.primal_residual
                <= self.config.accepted_constraint_violation
                and solution.maximum_violation
                <= self.config.accepted_constraint_violation
                and np.all(np.isfinite(solution.decision))
            )
            if accepted:
                self.accepted_solve_count += 1
                self.converged_solve_count += int(solution.converged)
                self.planned_ax_mps2 = float(solution.decision[0])
                self.planned_az_mps2 = float(
                    solution.decision[self.config.horizon_steps]
                )
                self.previous_ax_mps2 = self.planned_ax_mps2
                self.previous_az_mps2 = self.planned_az_mps2
                self.warm_decision = shift_warm_start(
                    solution.decision,
                    self.config.horizon_steps,
                )
            else:
                self.fallback_count += 1
                self.warm_decision = None
            self.latest_diagnostics = {
                **plan_diagnostics,
                "optimizer_converged": float(solution.converged),
                "optimizer_solution_accepted": float(accepted),
                "optimizer_fallback": float(not accepted),
                "optimizer_iterations": float(solution.iterations),
                "optimizer_primal_residual": solution.primal_residual,
                "optimizer_dual_residual": solution.dual_residual,
                "optimizer_max_constraint_violation": solution.maximum_violation,
                "optimizer_objective": solution.objective,
            }
            self.next_replan_time_s = state.time_s + self.config.replan_period_s

        diagnostics = {
            **self.latest_diagnostics,
            "optimizer_replanned": float(replanned),
            "optimizer_terminal_handoff": 0.0,
        }
        if diagnostics["optimizer_fallback"] > 0.5:
            fallback = corridor_guidance_command(
                state,
                vehicle,
                environment,
                guidance,
                attitude,
                target_x_m,
            )
            return PredictiveGuidanceResult(fallback, diagnostics)

        commanded = command_from_acceleration(
            state,
            vehicle,
            environment,
            attitude,
            self.planned_ax_mps2,
            self.planned_az_mps2,
            self.config.maximum_tilt_rad,
        )
        commanded = enforce_minimum_throttle_mode(
            commanded,
            state,
            vehicle,
            environment,
            guidance,
        )
        return PredictiveGuidanceResult(commanded, diagnostics)

    def summary(self) -> dict[str, float | int]:
        return {
            "optimizer_solve_count": self.solve_count,
            "optimizer_converged_solve_count": self.converged_solve_count,
            "optimizer_accepted_solve_count": self.accepted_solve_count,
            "optimizer_fallback_count": self.fallback_count,
            "optimizer_convergence_rate": (
                self.converged_solve_count / self.solve_count
                if self.solve_count
                else 0.0
            ),
            "optimizer_acceptance_rate": (
                self.accepted_solve_count / self.solve_count
                if self.solve_count
                else 0.0
            ),
            "optimizer_mean_iterations": (
                self.iteration_sum / self.solve_count if self.solve_count else 0.0
            ),
            "optimizer_maximum_observed_violation": self.maximum_observed_violation,
        }

    def _solve_plan(
        self,
        state: State,
        vehicle: Vehicle,
        environment: Environment,
        guidance: Guidance,
        target_x_m: float,
    ) -> tuple[QpSolution, dict[str, float]]:
        config = self.config
        horizon_s = choose_horizon(state, config)
        node_dt_s = horizon_s / config.horizon_steps
        matrices = prediction_matrices(config.horizon_steps, node_dt_s)
        problem = build_qp(
            state,
            vehicle,
            environment,
            guidance,
            config,
            target_x_m,
            horizon_s,
            node_dt_s,
            matrices,
            self.previous_ax_mps2,
            self.previous_az_mps2,
        )
        solution = solve_box_constrained_qp_admm(
            problem["quadratic"],
            problem["linear"],
            problem["constraint_matrix"],
            problem["lower_bounds"],
            problem["upper_bounds"],
            config,
            self.warm_decision,
        )
        diagnostics = evaluate_plan(
            solution.decision,
            state,
            vehicle,
            environment,
            config,
            target_x_m,
            horizon_s,
            matrices,
        )
        return solution, diagnostics

    @staticmethod
    def _empty_diagnostics() -> dict[str, float]:
        return {
            "optimizer_replanned": 0.0,
            "optimizer_terminal_handoff": 0.0,
            "optimizer_converged": 0.0,
            "optimizer_solution_accepted": 0.0,
            "optimizer_fallback": 1.0,
            "optimizer_iterations": 0.0,
            "optimizer_primal_residual": 0.0,
            "optimizer_dual_residual": 0.0,
            "optimizer_max_constraint_violation": 0.0,
            "optimizer_objective": 0.0,
            "optimizer_horizon_s": 0.0,
            "optimizer_predicted_terminal_x_error_m": 0.0,
            "optimizer_predicted_terminal_z_m": 0.0,
            "optimizer_predicted_terminal_vx_mps": 0.0,
            "optimizer_predicted_terminal_vz_mps": 0.0,
            "optimizer_minimum_tilt_margin_deg": 0.0,
            "optimizer_minimum_thrust_margin_mps2": 0.0,
            "optimizer_minimum_glideslope_margin_m": 0.0,
            "optimizer_tilt_constraint_active": 0.0,
            "optimizer_thrust_constraint_active": 0.0,
            "optimizer_glideslope_constraint_active": 0.0,
        }


def choose_horizon(state: State, config: PredictiveGuidanceConfig) -> float:
    closing_speed = max(1.0, -state.vz_mps - config.terminal_descent_mps)
    kinematic_time = 2.0 * max(state.z_m, 0.0) / closing_speed
    return clamp(
        kinematic_time,
        config.minimum_horizon_s,
        config.maximum_horizon_s,
    )


def prediction_matrices(steps: int, dt_s: float) -> dict[str, np.ndarray]:
    position = np.zeros((steps, steps), dtype=float)
    velocity = np.zeros((steps, steps), dtype=float)
    for row in range(steps):
        for column in range(row + 1):
            velocity[row, column] = dt_s
            position[row, column] = dt_s * dt_s * (row - column + 0.5)
    time = dt_s * np.arange(1, steps + 1, dtype=float)
    return {"position": position, "velocity": velocity, "time": time}


def build_qp(
    state: State,
    vehicle: Vehicle,
    environment: Environment,
    guidance: Guidance,
    config: PredictiveGuidanceConfig,
    target_x_m: float,
    horizon_s: float,
    node_dt_s: float,
    matrices: dict[str, np.ndarray],
    previous_ax_mps2: float,
    previous_az_mps2: float,
) -> dict[str, np.ndarray]:
    steps = config.horizon_steps
    variables = 2 * steps
    position = matrices["position"]
    velocity = matrices["velocity"]
    time = matrices["time"]
    zeros = np.zeros_like(position)
    horizontal_position_map = np.hstack((position, zeros))
    vertical_position_map = np.hstack((zeros, position))
    horizontal_velocity_map = np.hstack((velocity, zeros))
    vertical_velocity_map = np.hstack((zeros, velocity))

    x_base = state.x_m + state.vx_mps * time
    z_base = state.z_m + state.vz_mps * time
    vx_base = np.full(steps, state.vx_mps)
    vz_base = np.full(steps, state.vz_mps)
    x_reference, vx_reference = cubic_reference(
        state.x_m,
        state.vx_mps,
        target_x_m,
        0.0,
        time,
        horizon_s,
    )
    z_reference, vz_reference = cubic_reference(
        state.z_m,
        state.vz_mps,
        0.0,
        -config.terminal_descent_mps,
        time,
        horizon_s,
    )

    quadratic = np.eye(variables) * 1.0e-8
    linear = np.zeros(variables, dtype=float)

    def add_least_squares(
        mapping: np.ndarray,
        offset: np.ndarray,
        reference: np.ndarray,
        weights: np.ndarray,
    ) -> None:
        nonlocal quadratic, linear
        weighted_mapping = weights[:, None] * mapping
        weighted_error = weights * (offset - reference)
        quadratic += 2.0 * weighted_mapping.T @ weighted_mapping
        linear += 2.0 * weighted_mapping.T @ weighted_error

    progress = np.arange(1, steps + 1, dtype=float) / steps
    add_least_squares(
        horizontal_position_map,
        x_base,
        x_reference,
        np.sqrt(config.position_weight * (0.25 + progress)),
    )
    add_least_squares(
        horizontal_velocity_map,
        vx_base,
        vx_reference,
        np.sqrt(config.velocity_weight * (0.25 + progress)),
    )
    add_least_squares(
        vertical_position_map,
        z_base,
        z_reference,
        np.sqrt(config.vertical_position_weight * (0.25 + progress)),
    )
    add_least_squares(
        vertical_velocity_map,
        vz_base,
        vz_reference,
        np.sqrt(config.vertical_velocity_weight * (0.25 + progress)),
    )
    terminal_selector = np.zeros((1, steps), dtype=float)
    terminal_selector[0, -1] = 1.0
    add_least_squares(
        terminal_selector @ horizontal_position_map,
        np.array([x_base[-1]]),
        np.array([target_x_m]),
        np.array([math.sqrt(config.terminal_position_weight)]),
    )
    add_least_squares(
        terminal_selector @ horizontal_velocity_map,
        np.array([vx_base[-1]]),
        np.array([0.0]),
        np.array([math.sqrt(config.terminal_velocity_weight)]),
    )
    add_least_squares(
        terminal_selector @ vertical_position_map,
        np.array([z_base[-1]]),
        np.array([0.0]),
        np.array([math.sqrt(config.vertical_terminal_position_weight)]),
    )
    add_least_squares(
        terminal_selector @ vertical_velocity_map,
        np.array([vz_base[-1]]),
        np.array([-config.terminal_descent_mps]),
        np.array([math.sqrt(config.vertical_terminal_velocity_weight)]),
    )

    quadratic += 2.0 * config.acceleration_weight * np.eye(variables)
    vertical_slice = slice(steps, variables)
    quadratic[vertical_slice, vertical_slice] += (
        2.0 * config.thrust_regularization_weight * np.eye(steps)
    )
    linear[vertical_slice] += (
        2.0 * config.thrust_regularization_weight * environment.gravity_mps2
    )
    difference = first_difference_matrix(steps)
    slew_mapping = np.zeros((2 * steps, variables), dtype=float)
    slew_mapping[:steps, :steps] = difference
    slew_mapping[steps:, steps:] = difference
    slew_reference = np.zeros(2 * steps, dtype=float)
    slew_reference[0] = previous_ax_mps2
    slew_reference[steps] = previous_az_mps2
    add_least_squares(
        slew_mapping,
        np.zeros(2 * steps),
        slew_reference,
        np.full(2 * steps, math.sqrt(config.slew_weight)),
    )

    constraints: list[np.ndarray] = []
    lower_bounds: list[float] = []
    upper_bounds: list[float] = []

    def add_constraint(row: np.ndarray, lower: float, upper: float) -> None:
        constraints.append(row)
        lower_bounds.append(lower)
        upper_bounds.append(upper)

    maximum_specific_acceleration = vehicle.max_thrust_n / state.mass_kg
    vertical_lower = -environment.gravity_mps2 + 0.20
    vertical_upper = min(
        guidance.max_vertical_accel_mps2,
        maximum_specific_acceleration - environment.gravity_mps2,
    )
    for index in range(steps):
        row_ax = np.zeros(variables)
        row_ax[index] = 1.0
        add_constraint(
            row_ax,
            -guidance.max_lateral_accel_mps2,
            guidance.max_lateral_accel_mps2,
        )
        row_az = np.zeros(variables)
        row_az[steps + index] = 1.0
        add_constraint(
            row_az,
            vertical_lower,
            vertical_upper,
        )

        tangent_tilt = math.tan(config.maximum_tilt_rad)
        positive_tilt = np.zeros(variables)
        positive_tilt[index] = 1.0
        positive_tilt[steps + index] = -tangent_tilt
        add_constraint(
            positive_tilt,
            -math.inf,
            tangent_tilt * environment.gravity_mps2,
        )
        negative_tilt = np.zeros(variables)
        negative_tilt[index] = -1.0
        negative_tilt[steps + index] = -tangent_tilt
        add_constraint(
            negative_tilt,
            -math.inf,
            tangent_tilt * environment.gravity_mps2,
        )

        polygon_sides = 12
        inscribed_limit = maximum_specific_acceleration * math.cos(
            math.pi / polygon_sides
        )
        for side in range(polygon_sides):
            angle = 2.0 * math.pi * side / polygon_sides
            normal_x = math.cos(angle)
            normal_z = math.sin(angle)
            thrust_row = np.zeros(variables)
            thrust_row[index] = normal_x
            thrust_row[steps + index] = normal_z
            add_constraint(
                thrust_row,
                -math.inf,
                inscribed_limit - normal_z * environment.gravity_mps2,
            )

    difference = first_difference_matrix(steps)
    for index in range(steps):
        row_ax_slew = np.zeros(variables)
        row_ax_slew[:steps] = difference[index]
        ax_offset = previous_ax_mps2 if index == 0 else 0.0
        add_constraint(
            row_ax_slew,
            -config.maximum_ax_step_mps2 + ax_offset,
            config.maximum_ax_step_mps2 + ax_offset,
        )
        row_az_slew = np.zeros(variables)
        row_az_slew[steps:] = difference[index]
        az_offset = previous_az_mps2 if index == 0 else 0.0
        add_constraint(
            row_az_slew,
            -config.maximum_az_step_mps2 + az_offset,
            config.maximum_az_step_mps2 + az_offset,
        )

    for index in range(steps):
        altitude_row = vertical_position_map[index]
        add_constraint(altitude_row, -z_base[index], math.inf)
        positive_glide = (
            horizontal_position_map[index]
            - config.glide_slope_ratio * vertical_position_map[index]
        )
        positive_offset = (
            x_base[index]
            - target_x_m
            - config.glide_slope_ratio * z_base[index]
        )
        add_constraint(
            positive_glide,
            -math.inf,
            config.glide_slope_base_m - positive_offset,
        )
        negative_glide = (
            -horizontal_position_map[index]
            - config.glide_slope_ratio * vertical_position_map[index]
        )
        negative_offset = (
            -(x_base[index] - target_x_m)
            - config.glide_slope_ratio * z_base[index]
        )
        add_constraint(
            negative_glide,
            -math.inf,
            config.glide_slope_base_m - negative_offset,
        )

    return {
        "quadratic": quadratic,
        "linear": linear,
        "constraint_matrix": np.vstack(constraints),
        "lower_bounds": np.array(lower_bounds, dtype=float),
        "upper_bounds": np.array(upper_bounds, dtype=float),
    }


def solve_box_constrained_qp_admm(
    quadratic: np.ndarray,
    linear: np.ndarray,
    constraint_matrix: np.ndarray,
    lower_bounds: np.ndarray,
    upper_bounds: np.ndarray,
    config: PredictiveGuidanceConfig,
    warm_start: np.ndarray | None = None,
) -> QpSolution:
    variables = quadratic.shape[0]
    rho = config.admm_rho
    regularization = 1.0e-7
    system = (
        quadratic
        + rho * constraint_matrix.T @ constraint_matrix
        + regularization * np.eye(variables)
    )
    system_inverse = np.linalg.inv(system)
    decision = (
        warm_start.copy()
        if warm_start is not None and warm_start.shape == (variables,)
        else np.zeros(variables)
    )
    projected = np.clip(
        constraint_matrix @ decision,
        lower_bounds,
        upper_bounds,
    )
    scaled_dual = np.zeros_like(projected)
    primal_residual = math.inf
    dual_residual = math.inf
    iterations = 0
    for iterations in range(1, config.admm_max_iterations + 1):
        rhs = -linear + rho * constraint_matrix.T @ (projected - scaled_dual)
        decision = system_inverse @ rhs
        affine = constraint_matrix @ decision
        previous_projected = projected.copy()
        projected = np.clip(affine + scaled_dual, lower_bounds, upper_bounds)
        scaled_dual += affine - projected
        primal_residual = float(np.max(np.abs(affine - projected)))
        dual_residual = float(
            rho
            * np.max(
                np.abs(
                    constraint_matrix.T @ (projected - previous_projected)
                )
            )
        )
        if (
            primal_residual <= config.admm_absolute_tolerance
            and dual_residual <= config.admm_absolute_tolerance
        ):
            break

    affine = constraint_matrix @ decision
    lower_violation = np.maximum(lower_bounds - affine, 0.0)
    upper_violation = np.maximum(affine - upper_bounds, 0.0)
    maximum_violation = float(
        max(np.max(lower_violation), np.max(upper_violation))
    )
    objective = float(
        0.5 * decision.T @ quadratic @ decision + linear.T @ decision
    )
    return QpSolution(
        decision=decision,
        iterations=iterations,
        converged=(
            primal_residual <= config.admm_absolute_tolerance
            and dual_residual <= config.admm_absolute_tolerance
        ),
        primal_residual=primal_residual,
        dual_residual=dual_residual,
        maximum_violation=maximum_violation,
        objective=objective,
    )


def evaluate_plan(
    decision: np.ndarray,
    state: State,
    vehicle: Vehicle,
    environment: Environment,
    config: PredictiveGuidanceConfig,
    target_x_m: float,
    horizon_s: float,
    matrices: dict[str, np.ndarray],
) -> dict[str, float]:
    steps = config.horizon_steps
    ax = decision[:steps]
    az = decision[steps:]
    time = matrices["time"]
    position = matrices["position"]
    velocity = matrices["velocity"]
    x = state.x_m + state.vx_mps * time + position @ ax
    z = state.z_m + state.vz_mps * time + position @ az
    vx = state.vx_mps + velocity @ ax
    vz = state.vz_mps + velocity @ az
    specific_vertical = environment.gravity_mps2 + az
    tilt = np.abs(np.arctan2(ax, np.maximum(specific_vertical, 0.1)))
    thrust_acceleration = np.hypot(ax, specific_vertical)
    thrust_limit = vehicle.max_thrust_n / state.mass_kg
    glide_margin = (
        config.glide_slope_base_m
        + config.glide_slope_ratio * z
        - np.abs(x - target_x_m)
    )
    minimum_tilt_margin = float(config.maximum_tilt_rad - np.max(tilt))
    minimum_thrust_margin = float(thrust_limit - np.max(thrust_acceleration))
    minimum_glide_margin = float(np.min(glide_margin))
    return {
        "optimizer_horizon_s": horizon_s,
        "optimizer_predicted_terminal_x_error_m": float(x[-1] - target_x_m),
        "optimizer_predicted_terminal_z_m": float(z[-1]),
        "optimizer_predicted_terminal_vx_mps": float(vx[-1]),
        "optimizer_predicted_terminal_vz_mps": float(vz[-1]),
        "optimizer_minimum_tilt_margin_deg": math.degrees(minimum_tilt_margin),
        "optimizer_minimum_thrust_margin_mps2": minimum_thrust_margin,
        "optimizer_minimum_glideslope_margin_m": minimum_glide_margin,
        "optimizer_tilt_constraint_active": float(
            minimum_tilt_margin < math.radians(0.30)
        ),
        "optimizer_thrust_constraint_active": float(
            minimum_thrust_margin < 0.35
        ),
        "optimizer_glideslope_constraint_active": float(
            minimum_glide_margin < 0.35
        ),
    }


def cubic_reference(
    start_position: float,
    start_velocity: float,
    end_position: float,
    end_velocity: float,
    times_s: np.ndarray,
    duration_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    normalized = np.clip(times_s / duration_s, 0.0, 1.0)
    h00 = 2.0 * normalized**3 - 3.0 * normalized**2 + 1.0
    h10 = normalized**3 - 2.0 * normalized**2 + normalized
    h01 = -2.0 * normalized**3 + 3.0 * normalized**2
    h11 = normalized**3 - normalized**2
    position = (
        h00 * start_position
        + h10 * duration_s * start_velocity
        + h01 * end_position
        + h11 * duration_s * end_velocity
    )
    dh00 = (6.0 * normalized**2 - 6.0 * normalized) / duration_s
    dh10 = 3.0 * normalized**2 - 4.0 * normalized + 1.0
    dh01 = (-6.0 * normalized**2 + 6.0 * normalized) / duration_s
    dh11 = 3.0 * normalized**2 - 2.0 * normalized
    velocity = (
        dh00 * start_position
        + dh10 * start_velocity
        + dh01 * end_position
        + dh11 * end_velocity
    )
    return position, velocity


def first_difference_matrix(size: int) -> np.ndarray:
    difference = np.eye(size)
    for index in range(1, size):
        difference[index, index - 1] = -1.0
    return difference


def shift_warm_start(decision: np.ndarray, steps: int) -> np.ndarray:
    shifted = np.empty_like(decision)
    shifted[: steps - 1] = decision[1:steps]
    shifted[steps - 1] = decision[steps - 1]
    shifted[steps : 2 * steps - 1] = decision[steps + 1 :]
    shifted[2 * steps - 1] = decision[2 * steps - 1]
    return shifted


def enforce_minimum_throttle_mode(
    command: Command,
    state: State,
    vehicle: Vehicle,
    environment: Environment,
    guidance: Guidance,
) -> Command:
    angle_for_throttle = max(0.25, math.cos(state.theta_rad))
    relaxed_throttle = (
        state.mass_kg
        * max(0.0, environment.gravity_mps2 + command.desired_az_mps2)
        / angle_for_throttle
        / vehicle.max_thrust_n
    )
    if not (0.0 < relaxed_throttle < vehicle.min_throttle) or state.z_m <= 3.0:
        return command
    velocity_reference = vertical_velocity_reference(state.z_m, guidance)
    if state.z_m < 90.0:
        velocity_reference = max(velocity_reference, -1.35)
    if state.z_m < 25.0:
        velocity_reference = max(velocity_reference, -0.75)
    throttle = (
        vehicle.min_throttle
        if state.vz_mps < velocity_reference - 0.10
        else 0.0
    )
    return Command(
        throttle=throttle,
        gimbal_rad=command.gimbal_rad,
        desired_theta_rad=command.desired_theta_rad,
        desired_ax_mps2=command.desired_ax_mps2,
        desired_az_mps2=command.desired_az_mps2,
    )
