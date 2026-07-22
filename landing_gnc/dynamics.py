"""Planar powered-descent dynamics.

Coordinate convention:
    x: horizontal position, positive downrange
    z: altitude above landing pad
    theta: body tilt from vertical, positive tips thrust toward +x
    gimbal: engine deflection relative to body, positive rotates thrust toward +x
"""

import math

from .models import Command, Environment, State, Vehicle

G0 = 9.80665


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def derivatives(state: State, command: Command, vehicle: Vehicle, env: Environment) -> State:
    throttle = clamp(command.throttle, 0.0, 1.0)
    gimbal = clamp(command.gimbal_rad, -vehicle.max_gimbal_rad, vehicle.max_gimbal_rad)

    prop_remaining = max(0.0, state.mass_kg - vehicle.dry_mass_kg)
    thrust_n = throttle * vehicle.max_thrust_n if prop_remaining > 0.0 else 0.0
    thrust_angle = state.theta_rad

    tx = thrust_n * math.sin(thrust_angle)
    tz = thrust_n * math.cos(thrust_angle)

    vrel_x = state.vx_mps - env.wind_x_mps
    vrel_z = state.vz_mps
    vrel = math.hypot(vrel_x, vrel_z)
    if vrel > 1.0e-9:
        drag_mag = 0.5 * env.air_density_kg_m3 * vrel * vrel * vehicle.reference_area_m2 * vehicle.drag_coefficient
        drag_x = -drag_mag * vrel_x / vrel
        drag_z = -drag_mag * vrel_z / vrel
    else:
        drag_x = 0.0
        drag_z = 0.0

    ax = (tx + drag_x) / state.mass_kg
    az = (tz + drag_z) / state.mass_kg - env.gravity_mps2

    torque = thrust_n * vehicle.engine_moment_arm_m * math.sin(gimbal)
    omega_dot = (torque - vehicle.rotational_damping_nms * state.omega_radps) / vehicle.inertia_kg_m2
    mdot = -thrust_n / (vehicle.isp_s * G0) if prop_remaining > 0.0 else 0.0

    return State(
        time_s=1.0,
        x_m=state.vx_mps,
        z_m=state.vz_mps,
        vx_mps=ax,
        vz_mps=az,
        theta_rad=state.omega_radps,
        omega_radps=omega_dot,
        mass_kg=mdot,
    )


def add_scaled(state: State, delta: State, scale: float) -> State:
    return State(
        time_s=state.time_s + scale * delta.time_s,
        x_m=state.x_m + scale * delta.x_m,
        z_m=state.z_m + scale * delta.z_m,
        vx_mps=state.vx_mps + scale * delta.vx_mps,
        vz_mps=state.vz_mps + scale * delta.vz_mps,
        theta_rad=state.theta_rad + scale * delta.theta_rad,
        omega_radps=state.omega_radps + scale * delta.omega_radps,
        mass_kg=state.mass_kg + scale * delta.mass_kg,
    )


def rk4_step(state: State, command: Command, vehicle: Vehicle, env: Environment, dt_s: float) -> State:
    k1 = derivatives(state, command, vehicle, env)
    k2 = derivatives(add_scaled(state, k1, 0.5 * dt_s), command, vehicle, env)
    k3 = derivatives(add_scaled(state, k2, 0.5 * dt_s), command, vehicle, env)
    k4 = derivatives(add_scaled(state, k3, dt_s), command, vehicle, env)

    next_state = State(
        time_s=state.time_s + dt_s,
        x_m=state.x_m + dt_s / 6.0 * (k1.x_m + 2.0 * k2.x_m + 2.0 * k3.x_m + k4.x_m),
        z_m=state.z_m + dt_s / 6.0 * (k1.z_m + 2.0 * k2.z_m + 2.0 * k3.z_m + k4.z_m),
        vx_mps=state.vx_mps + dt_s / 6.0 * (k1.vx_mps + 2.0 * k2.vx_mps + 2.0 * k3.vx_mps + k4.vx_mps),
        vz_mps=state.vz_mps + dt_s / 6.0 * (k1.vz_mps + 2.0 * k2.vz_mps + 2.0 * k3.vz_mps + k4.vz_mps),
        theta_rad=state.theta_rad + dt_s / 6.0 * (k1.theta_rad + 2.0 * k2.theta_rad + 2.0 * k3.theta_rad + k4.theta_rad),
        omega_radps=state.omega_radps + dt_s / 6.0 * (k1.omega_radps + 2.0 * k2.omega_radps + 2.0 * k3.omega_radps + k4.omega_radps),
        mass_kg=state.mass_kg + dt_s / 6.0 * (k1.mass_kg + 2.0 * k2.mass_kg + 2.0 * k3.mass_kg + k4.mass_kg),
    )
    if next_state.mass_kg < vehicle.dry_mass_kg:
        next_state.mass_kg = vehicle.dry_mass_kg
    return next_state
