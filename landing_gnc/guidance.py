"""Guidance and control laws for the initial powered-landing baseline."""

import math

from .dynamics import clamp
from .models import AttitudeControl, Command, Environment, Guidance, State, Vehicle


def vertical_velocity_reference(altitude_m: float, guidance: Guidance) -> float:
    altitude = max(altitude_m, 0.0)
    braking_speed = math.sqrt(max(0.0, 2.0 * guidance.vertical_decel_mps2 * altitude))
    return -max(guidance.terminal_descent_mps, min(85.0, braking_speed))


def guidance_command(
    state: State,
    vehicle: Vehicle,
    env: Environment,
    guidance: Guidance,
    attitude: AttitudeControl,
) -> Command:
    v_ref = vertical_velocity_reference(state.z_m, guidance)
    az_cmd = guidance.vertical_kv * (v_ref - state.vz_mps)
    az_cmd = clamp(az_cmd, -env.gravity_mps2 + 0.5, guidance.max_vertical_accel_mps2)

    ax_cmd = -guidance.lateral_kp * state.x_m - guidance.lateral_kd * state.vx_mps
    ax_cmd = clamp(ax_cmd, -guidance.max_lateral_accel_mps2, guidance.max_lateral_accel_mps2)

    return command_from_acceleration(state, vehicle, env, attitude, ax_cmd, az_cmd, max_tilt_rad=0.14)


def corridor_guidance_command(
    state: State,
    vehicle: Vehicle,
    env: Environment,
    guidance: Guidance,
    attitude: AttitudeControl,
) -> Command:
    """Altitude-scheduled terminal guidance with an explicit landing corridor.

    The law remains intentionally simple. It improves the baseline by correcting
    lateral error earlier while reducing late tilt demand when vertical braking
    margin is most valuable.
    """
    corridor_half_width_m = 0.65 + 0.020 * max(state.z_m, 0.0)
    corridor_error = 0.0
    if abs(state.x_m) > corridor_half_width_m:
        corridor_error = state.x_m - math.copysign(corridor_half_width_m, state.x_m)

    altitude_scale = clamp(state.z_m / 180.0, 0.28, 1.0)
    ax_cmd = (
        -0.030 * state.x_m
        -0.52 * state.vx_mps
        -0.060 * corridor_error
    ) * altitude_scale
    ax_cmd = clamp(ax_cmd, -guidance.max_lateral_accel_mps2, guidance.max_lateral_accel_mps2)

    v_ref = vertical_velocity_reference(state.z_m, guidance)
    if state.z_m < 90.0:
        v_ref = max(v_ref, -1.35)
    if state.z_m < 25.0:
        v_ref = max(v_ref, -0.75)

    vertical_gain = 2.45 if state.z_m < 120.0 else guidance.vertical_kv
    az_cmd = vertical_gain * (v_ref - state.vz_mps)
    az_cmd = clamp(az_cmd, -env.gravity_mps2 + 0.5, guidance.max_vertical_accel_mps2)

    max_tilt_deg = 2.2 + 4.8 * clamp(state.z_m / 260.0, 0.0, 1.0)
    if state.z_m < 70.0 and state.vz_mps < -2.0:
        max_tilt_deg *= 0.72
    return command_from_acceleration(
        state,
        vehicle,
        env,
        attitude,
        ax_cmd,
        az_cmd,
        max_tilt_rad=math.radians(max_tilt_deg),
    )


def command_from_acceleration(
    state: State,
    vehicle: Vehicle,
    env: Environment,
    attitude: AttitudeControl,
    ax_cmd: float,
    az_cmd: float,
    max_tilt_rad: float,
) -> Command:
    required_vertical = env.gravity_mps2 + az_cmd
    desired_thrust_angle = math.atan2(ax_cmd, max(required_vertical, 0.25))
    desired_theta = clamp(desired_thrust_angle, -max_tilt_rad, max_tilt_rad)

    gimbal = clamp(
        attitude.kp * (desired_theta - state.theta_rad) - attitude.kd * state.omega_radps,
        -vehicle.max_gimbal_rad,
        vehicle.max_gimbal_rad,
    )

    angle_for_throttle = max(0.25, math.cos(state.theta_rad))
    required_thrust = state.mass_kg * required_vertical / angle_for_throttle
    throttle = required_thrust / vehicle.max_thrust_n
    if state.z_m > 3.0:
        throttle = clamp(throttle, 0.0, 1.0)
        if throttle < vehicle.min_throttle:
            throttle = 0.0
    else:
        throttle = clamp(throttle, 0.0, 1.0)

    return Command(
        throttle=throttle,
        gimbal_rad=gimbal,
        desired_theta_rad=desired_theta,
        desired_ax_mps2=ax_cmd,
        desired_az_mps2=az_cmd,
    )
