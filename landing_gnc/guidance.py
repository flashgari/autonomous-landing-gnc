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

    required_vertical = env.gravity_mps2 + az_cmd
    desired_thrust_angle = math.atan2(ax_cmd, max(required_vertical, 0.25))
    desired_theta = clamp(desired_thrust_angle, -0.14, 0.14)

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
