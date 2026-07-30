"""3D landing guidance, quaternion attitude control, and engine allocation."""

from __future__ import annotations

from collections import deque

import numpy as np

from .actuators import first_order_step, rate_limit
from .sixdof_math import (
    desired_attitude_from_thrust,
    quaternion_error_vector,
    quaternion_to_matrix,
)
from .sixdof_models import (
    AchievedActuation,
    EngineCommand,
    SixDofActuatorModel,
    SixDofAttitudeControl,
    SixDofEnvironment,
    SixDofGuidance,
    SixDofState,
    SixDofVehicle,
    WrenchCommand,
)


def vertical_velocity_reference_3d(
    altitude_m: float,
    guidance: SixDofGuidance,
) -> float:
    altitude = max(altitude_m, 0.0)
    braking_speed = np.sqrt(
        max(0.0, 2.0 * guidance.vertical_deceleration_mps2 * altitude)
    )
    return -max(
        guidance.terminal_descent_mps,
        min(85.0, float(braking_speed)),
    )


def guidance_acceleration(
    state: SixDofState,
    environment: SixDofEnvironment,
    guidance: SixDofGuidance,
    target_inertial_m: np.ndarray,
) -> np.ndarray:
    position_error_horizontal = (
        state.position_inertial_m[:2] - target_inertial_m[:2]
    )
    horizontal_distance = float(np.linalg.norm(position_error_horizontal))
    corridor_half_width = 0.65 + 0.020 * max(
        state.position_inertial_m[2],
        0.0,
    )
    corridor_error = np.zeros(2)
    if horizontal_distance > corridor_half_width:
        corridor_error = (
            position_error_horizontal
            * (horizontal_distance - corridor_half_width)
            / horizontal_distance
        )

    altitude_scale = float(
        np.clip(state.position_inertial_m[2] / 180.0, 0.28, 1.0)
    )
    horizontal_acceleration = (
        -guidance.horizontal_position_gain * position_error_horizontal
        - guidance.horizontal_velocity_gain * state.velocity_inertial_mps[:2]
        - guidance.corridor_gain * corridor_error
    ) * altitude_scale
    horizontal_norm = float(np.linalg.norm(horizontal_acceleration))
    if horizontal_norm > guidance.maximum_horizontal_acceleration_mps2:
        horizontal_acceleration *= (
            guidance.maximum_horizontal_acceleration_mps2 / horizontal_norm
        )

    vertical_reference = vertical_velocity_reference_3d(
        state.position_inertial_m[2],
        guidance,
    )
    if (
        state.position_inertial_m[2]
        < guidance.approach_gate_altitude_m
    ):
        vertical_reference = max(
            vertical_reference,
            -guidance.approach_descent_mps,
        )
    if (
        state.position_inertial_m[2]
        < guidance.flare_gate_altitude_m
    ):
        vertical_reference = max(
            vertical_reference,
            -guidance.flare_descent_mps,
        )
    if (
        state.position_inertial_m[2]
        < guidance.terminal_gate_altitude_m
    ):
        vertical_reference = max(
            vertical_reference,
            -guidance.terminal_descent_mps,
        )
    vertical_gain = (
        2.55
        if state.position_inertial_m[2] < 120.0
        else guidance.vertical_velocity_gain
    )
    vertical_acceleration = vertical_gain * (
        vertical_reference - state.velocity_inertial_mps[2]
    )
    vertical_acceleration = float(
        np.clip(
            vertical_acceleration,
            -environment.gravity_mps2 + 0.5,
            guidance.maximum_vertical_acceleration_mps2,
        )
    )
    return np.array(
        [
            horizontal_acceleration[0],
            horizontal_acceleration[1],
            vertical_acceleration,
        ]
    )


def wrench_command(
    state: SixDofState,
    vehicle: SixDofVehicle,
    environment: SixDofEnvironment,
    guidance: SixDofGuidance,
    attitude_control: SixDofAttitudeControl,
    target_inertial_m: np.ndarray,
    desired_acceleration_override: np.ndarray | None = None,
) -> WrenchCommand:
    desired_acceleration = (
        np.asarray(desired_acceleration_override, dtype=float).copy()
        if desired_acceleration_override is not None
        else guidance_acceleration(
            state,
            environment,
            guidance,
            target_inertial_m,
        )
    )
    gravity = np.array([0.0, 0.0, -environment.gravity_mps2])
    desired_force_inertial = state.mass_kg * (
        desired_acceleration - gravity
    )

    vertical_force = max(1.0, desired_force_inertial[2])
    maximum_lateral_force = vertical_force * np.tan(
        guidance.maximum_tilt_rad
    )
    lateral_force = desired_force_inertial[:2]
    lateral_norm = float(np.linalg.norm(lateral_force))
    if lateral_norm > maximum_lateral_force:
        desired_force_inertial[:2] *= maximum_lateral_force / lateral_norm

    desired_quaternion = desired_attitude_from_thrust(
        desired_force_inertial,
        guidance.yaw_reference_rad,
    )
    attitude_error = quaternion_error_vector(
        state.quaternion_body_to_inertial,
        desired_quaternion,
    )
    proportional_gain = np.asarray(
        attitude_control.proportional_gain_nm,
        dtype=float,
    )
    derivative_gain = np.asarray(
        attitude_control.derivative_gain_nms,
        dtype=float,
    )
    inertia = vehicle.inertia_diag_kg_m2(state.mass_kg)
    omega = state.angular_velocity_body_radps
    gyro_compensation = np.cross(omega, inertia * omega)
    torque_command = (
        proportional_gain * attitude_error
        - derivative_gain * omega
        + gyro_compensation
    )
    torque_limit = np.asarray(
        attitude_control.maximum_torque_nm,
        dtype=float,
    )
    torque_command = np.clip(
        torque_command,
        -torque_limit,
        torque_limit,
    )
    return WrenchCommand(
        force_inertial_n=desired_force_inertial,
        torque_body_nm=torque_command,
        desired_quaternion_body_to_inertial=desired_quaternion,
        desired_acceleration_inertial_mps2=desired_acceleration,
    )


def engine_force_body(
    command: EngineCommand,
    vehicle: SixDofVehicle,
) -> np.ndarray:
    if not command.enabled or command.throttle <= 0.0:
        return np.zeros(3)
    direction = np.array(
        [
            np.tan(command.gimbal_x_rad),
            np.tan(command.gimbal_y_rad),
            1.0,
        ]
    )
    direction /= np.linalg.norm(direction)
    return (
        command.throttle
        * vehicle.maximum_engine_thrust_n
        * direction
    )


def wrench_from_engine_commands(
    commands: tuple[EngineCommand, ...],
    vehicle: SixDofVehicle,
    mass_kg: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    total_force = np.zeros(3)
    total_torque = np.zeros(3)
    total_thrust = 0.0
    for position, command in zip(
        vehicle.engine_positions_body_m(mass_kg),
        commands,
    ):
        force = engine_force_body(command, vehicle)
        total_force += force
        total_torque += np.cross(position, force)
        total_thrust += float(np.linalg.norm(force))
    return total_force, total_torque, total_thrust


def _allocation_matrix(
    engine_positions: tuple[np.ndarray, ...],
) -> np.ndarray:
    matrix = np.zeros((6, 3 * len(engine_positions)))
    for index, position in enumerate(engine_positions):
        start = 3 * index
        matrix[:3, start : start + 3] = np.eye(3)
        x, y, z = position
        matrix[3:, start : start + 3] = np.array(
            [
                [0.0, -z, y],
                [z, 0.0, -x],
                [-y, x, 0.0],
            ]
        )
    return matrix


def allocate_engine_commands(
    state: SixDofState,
    wrench: WrenchCommand,
    vehicle: SixDofVehicle,
    failed_engine_index: int | None = None,
) -> AchievedActuation:
    rotation = quaternion_to_matrix(
        state.quaternion_body_to_inertial
    )
    requested_force_body = rotation.T @ wrench.force_inertial_n
    requested_wrench = np.concatenate(
        (requested_force_body, wrench.torque_body_nm)
    )
    positions = vehicle.engine_positions_body_m(state.mass_kg)
    allocation = _allocation_matrix(positions)

    active = [
        index
        for index in range(len(positions))
        if index != failed_engine_index
    ]
    active_columns = [
        column
        for engine_index in active
        for column in range(3 * engine_index, 3 * engine_index + 3)
    ]
    active_matrix = allocation[:, active_columns]

    axial_reference = max(0.0, requested_force_body[2]) / max(
        1,
        len(active),
    )
    reference = np.zeros(len(active_columns))
    for active_index in range(len(active)):
        reference[3 * active_index + 2] = axial_reference

    characteristic_arm = max(
        vehicle.wet_engine_arm_m,
        vehicle.dry_engine_arm_m,
    )
    row_weight = np.diag(
        [
            1.0,
            1.0,
            1.0,
            1.0 / characteristic_arm,
            1.0 / characteristic_arm,
            1.0 / vehicle.engine_cluster_radius_m,
        ]
    )
    regularization = 0.015
    augmented_matrix = np.vstack(
        (
            row_weight @ active_matrix,
            np.sqrt(regularization) * np.eye(len(active_columns)),
        )
    )
    augmented_target = np.concatenate(
        (
            row_weight @ requested_wrench,
            np.sqrt(regularization) * reference,
        )
    )
    active_forces, *_ = np.linalg.lstsq(
        augmented_matrix,
        augmented_target,
        rcond=None,
    )

    force_components = np.zeros((len(positions), 3))
    for active_index, engine_index in enumerate(active):
        force_components[engine_index] = active_forces[
            3 * active_index : 3 * active_index + 3
        ]

    commands: list[EngineCommand] = []
    gimbal_saturated = 0
    throttle_saturated = 0
    for index, components in enumerate(force_components):
        enabled = index != failed_engine_index
        if not enabled:
            commands.append(EngineCommand(0.0, 0.0, 0.0, False))
            continue
        lateral = components[:2]
        axial = max(0.0, float(components[2]))
        lateral_norm = float(np.linalg.norm(lateral))
        maximum_lateral = axial * np.tan(vehicle.maximum_gimbal_rad)
        if lateral_norm > maximum_lateral and lateral_norm > 0.0:
            lateral *= maximum_lateral / lateral_norm
            gimbal_saturated += 1
        projected = np.array([lateral[0], lateral[1], axial])
        thrust = float(np.linalg.norm(projected))
        if thrust > vehicle.maximum_engine_thrust_n:
            projected *= vehicle.maximum_engine_thrust_n / thrust
            thrust = vehicle.maximum_engine_thrust_n
            throttle_saturated += 1
        throttle = thrust / vehicle.maximum_engine_thrust_n
        if 0.0 < throttle < vehicle.minimum_engine_throttle:
            throttle = (
                vehicle.minimum_engine_throttle
                if throttle >= 0.5 * vehicle.minimum_engine_throttle
                else 0.0
            )
        gimbal_x = (
            float(np.arctan2(projected[0], max(projected[2], 1.0e-12)))
            if throttle > 0.0
            else 0.0
        )
        gimbal_y = (
            float(np.arctan2(projected[1], max(projected[2], 1.0e-12)))
            if throttle > 0.0
            else 0.0
        )
        commands.append(
            EngineCommand(
                throttle=float(np.clip(throttle, 0.0, 1.0)),
                gimbal_x_rad=float(
                    np.clip(
                        gimbal_x,
                        -vehicle.maximum_gimbal_rad,
                        vehicle.maximum_gimbal_rad,
                    )
                ),
                gimbal_y_rad=float(
                    np.clip(
                        gimbal_y,
                        -vehicle.maximum_gimbal_rad,
                        vehicle.maximum_gimbal_rad,
                    )
                ),
                enabled=True,
            )
        )

    command_tuple = tuple(commands)
    achieved_force, achieved_torque, total_thrust = (
        wrench_from_engine_commands(
            command_tuple,
            vehicle,
            state.mass_kg,
        )
    )
    force_residual = requested_force_body - achieved_force
    torque_residual = wrench.torque_body_nm - achieved_torque
    normalized_residual = float(
        np.sqrt(
            np.dot(force_residual, force_residual)
            / vehicle.maximum_total_thrust_n**2
            + np.dot(torque_residual, torque_residual)
            / (
                vehicle.maximum_total_thrust_n
                * characteristic_arm
            )
            ** 2
        )
    )
    return AchievedActuation(
        engine_commands=command_tuple,
        force_body_n=achieved_force,
        torque_body_nm=achieved_torque,
        total_thrust_n=total_thrust,
        force_residual_n=force_residual,
        torque_residual_nm=torque_residual,
        normalized_allocation_residual=normalized_residual,
        gimbal_saturated_fraction=gimbal_saturated
        / max(1, len(active)),
        throttle_saturated_fraction=throttle_saturated
        / max(1, len(active)),
    )


class SixDofActuatorStack:
    """Delay, first-order lag, and rate limits for four engine commands."""

    def __init__(
        self,
        model: SixDofActuatorModel,
        vehicle: SixDofVehicle,
        dt_s: float,
    ) -> None:
        self.model = model
        self.vehicle = vehicle
        zero = tuple(
            EngineCommand(0.0, 0.0, 0.0, True)
            for _ in range(4)
        )
        delay_steps = max(
            0,
            round(model.command_delay_s / dt_s),
        )
        self.buffer = deque(
            [zero] * (delay_steps + 1),
            maxlen=delay_steps + 1,
        )
        self.state = list(zero)

    def step(
        self,
        requested: AchievedActuation,
        state: SixDofState,
        wrench: WrenchCommand,
        dt_s: float,
    ) -> AchievedActuation:
        self.buffer.append(requested.engine_commands)
        delayed = self.buffer[0]
        updated: list[EngineCommand] = []
        for current, target in zip(self.state, delayed):
            if not target.enabled:
                updated.append(
                    EngineCommand(0.0, 0.0, 0.0, False)
                )
                continue
            throttle_lagged = first_order_step(
                current.throttle,
                target.throttle,
                self.model.throttle_time_constant_s,
                dt_s,
            )
            gimbal_x_lagged = first_order_step(
                current.gimbal_x_rad,
                target.gimbal_x_rad,
                self.model.gimbal_time_constant_s,
                dt_s,
            )
            gimbal_y_lagged = first_order_step(
                current.gimbal_y_rad,
                target.gimbal_y_rad,
                self.model.gimbal_time_constant_s,
                dt_s,
            )
            updated.append(
                EngineCommand(
                    throttle=float(
                        np.clip(
                            rate_limit(
                                current.throttle,
                                throttle_lagged,
                                self.model.throttle_rate_limit_per_s,
                                dt_s,
                            ),
                            0.0,
                            1.0,
                        )
                    ),
                    gimbal_x_rad=float(
                        np.clip(
                            rate_limit(
                                current.gimbal_x_rad,
                                gimbal_x_lagged,
                                self.model.gimbal_rate_limit_radps,
                                dt_s,
                            ),
                            -self.vehicle.maximum_gimbal_rad,
                            self.vehicle.maximum_gimbal_rad,
                        )
                    ),
                    gimbal_y_rad=float(
                        np.clip(
                            rate_limit(
                                current.gimbal_y_rad,
                                gimbal_y_lagged,
                                self.model.gimbal_rate_limit_radps,
                                dt_s,
                            ),
                            -self.vehicle.maximum_gimbal_rad,
                            self.vehicle.maximum_gimbal_rad,
                        )
                    ),
                    enabled=True,
                )
            )
        self.state = updated
        command_tuple = tuple(updated)
        achieved_force, achieved_torque, total_thrust = (
            wrench_from_engine_commands(
                command_tuple,
                self.vehicle,
                state.mass_kg,
            )
        )
        rotation = quaternion_to_matrix(
            state.quaternion_body_to_inertial
        )
        requested_force_body = rotation.T @ wrench.force_inertial_n
        force_residual = requested_force_body - achieved_force
        torque_residual = wrench.torque_body_nm - achieved_torque
        arm = max(
            self.vehicle.wet_engine_arm_m,
            self.vehicle.dry_engine_arm_m,
        )
        normalized_residual = float(
            np.sqrt(
                np.dot(force_residual, force_residual)
                / self.vehicle.maximum_total_thrust_n**2
                + np.dot(torque_residual, torque_residual)
                / (
                    self.vehicle.maximum_total_thrust_n * arm
                )
                ** 2
            )
        )
        active = [command for command in command_tuple if command.enabled]
        gimbal_saturated = sum(
            np.hypot(
                command.gimbal_x_rad,
                command.gimbal_y_rad,
            )
            >= self.vehicle.maximum_gimbal_rad - 1.0e-5
            for command in active
        )
        throttle_saturated = sum(
            command.throttle >= 1.0 - 1.0e-5
            for command in active
        )
        return AchievedActuation(
            engine_commands=command_tuple,
            force_body_n=achieved_force,
            torque_body_nm=achieved_torque,
            total_thrust_n=total_thrust,
            force_residual_n=force_residual,
            torque_residual_nm=torque_residual,
            normalized_allocation_residual=normalized_residual,
            gimbal_saturated_fraction=gimbal_saturated
            / max(1, len(active)),
            throttle_saturated_fraction=throttle_saturated
            / max(1, len(active)),
        )
