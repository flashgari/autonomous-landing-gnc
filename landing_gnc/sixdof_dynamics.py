"""Nonlinear 6-DOF powered-descent dynamics.

Frame convention:
    inertial x/y: horizontal landing-plane coordinates
    inertial z: altitude, positive upward
    body +z: nominal thrust axis from engines toward the vehicle nose
    quaternion: rotates body-frame vectors into inertial coordinates
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .sixdof_math import quaternion_multiply, quaternion_normalize, quaternion_to_matrix
from .sixdof_models import (
    AchievedActuation,
    SixDofEnvironment,
    SixDofState,
    SixDofVehicle,
)

G0_MPS2 = 9.80665


@dataclass(frozen=True)
class SixDofDerivative:
    position_rate_mps: np.ndarray
    velocity_rate_mps2: np.ndarray
    quaternion_rate: np.ndarray
    angular_acceleration_body_radps2: np.ndarray
    mass_rate_kgps: float
    aerodynamic_force_body_n: np.ndarray
    aerodynamic_moment_body_nm: np.ndarray


def aerodynamic_loads(
    state: SixDofState,
    vehicle: SixDofVehicle,
    environment: SixDofEnvironment,
) -> tuple[np.ndarray, np.ndarray]:
    rotation = quaternion_to_matrix(state.quaternion_body_to_inertial)
    relative_velocity_inertial = (
        state.velocity_inertial_mps
        - np.asarray(environment.wind_inertial_mps, dtype=float)
    )
    speed = float(np.linalg.norm(relative_velocity_inertial))
    if speed < 1.0e-9:
        return np.zeros(3), np.zeros(3)

    relative_velocity_body = rotation.T @ relative_velocity_inertial
    dynamic_pressure = 0.5 * environment.air_density_kg_m3 * speed * speed
    drag_body = (
        -dynamic_pressure
        * vehicle.reference_area_m2
        * vehicle.drag_coefficient
        * relative_velocity_body
        / speed
    )

    crossflow = np.array(
        [relative_velocity_body[0], relative_velocity_body[1], 0.0]
    )
    crossflow_speed = float(np.linalg.norm(crossflow))
    if crossflow_speed > 1.0e-9:
        normal_force_body = (
            -dynamic_pressure
            * vehicle.reference_area_m2
            * vehicle.normal_force_slope
            * crossflow
            / speed
        )
    else:
        normal_force_body = np.zeros(3)

    force_body = drag_body + normal_force_body
    center_of_pressure_body = np.array(
        [0.0, 0.0, vehicle.center_of_pressure_offset_m]
    )
    moment_body = np.cross(center_of_pressure_body, normal_force_body)
    return force_body, moment_body


def derivatives(
    state: SixDofState,
    actuation: AchievedActuation,
    vehicle: SixDofVehicle,
    environment: SixDofEnvironment,
) -> SixDofDerivative:
    rotation = quaternion_to_matrix(state.quaternion_body_to_inertial)
    aero_force_body, aero_moment_body = aerodynamic_loads(
        state,
        vehicle,
        environment,
    )
    force_inertial = rotation @ (actuation.force_body_n + aero_force_body)
    gravity_inertial = np.array([0.0, 0.0, -environment.gravity_mps2])
    acceleration_inertial = force_inertial / state.mass_kg + gravity_inertial

    propellant_remaining = max(0.0, state.mass_kg - vehicle.dry_mass_kg)
    mass_rate = (
        -actuation.total_thrust_n / (vehicle.specific_impulse_s * G0_MPS2)
        if propellant_remaining > 0.0
        else 0.0
    )
    omega = state.angular_velocity_body_radps
    inertia_diag = vehicle.inertia_diag_kg_m2(state.mass_kg)
    inertia_rate_diag = vehicle.inertia_rate_diag_kg_m2_s(mass_rate)
    angular_momentum = inertia_diag * omega
    damping = np.asarray(vehicle.rotational_damping_nms, dtype=float) * omega
    net_moment = actuation.torque_body_nm + aero_moment_body - damping
    angular_acceleration = (
        net_moment
        - inertia_rate_diag * omega
        - np.cross(omega, angular_momentum)
    ) / inertia_diag

    quaternion_rate = 0.5 * quaternion_multiply(
        state.quaternion_body_to_inertial,
        np.array([0.0, omega[0], omega[1], omega[2]]),
    )
    return SixDofDerivative(
        position_rate_mps=state.velocity_inertial_mps,
        velocity_rate_mps2=acceleration_inertial,
        quaternion_rate=quaternion_rate,
        angular_acceleration_body_radps2=angular_acceleration,
        mass_rate_kgps=mass_rate,
        aerodynamic_force_body_n=aero_force_body,
        aerodynamic_moment_body_nm=aero_moment_body,
    )


def add_scaled(
    state: SixDofState,
    derivative: SixDofDerivative,
    scale: float,
) -> SixDofState:
    return SixDofState(
        time_s=state.time_s + scale,
        position_inertial_m=state.position_inertial_m
        + scale * derivative.position_rate_mps,
        velocity_inertial_mps=state.velocity_inertial_mps
        + scale * derivative.velocity_rate_mps2,
        quaternion_body_to_inertial=quaternion_normalize(
            state.quaternion_body_to_inertial
            + scale * derivative.quaternion_rate
        ),
        angular_velocity_body_radps=state.angular_velocity_body_radps
        + scale * derivative.angular_acceleration_body_radps2,
        mass_kg=state.mass_kg + scale * derivative.mass_rate_kgps,
    )


def rk4_step(
    state: SixDofState,
    actuation: AchievedActuation,
    vehicle: SixDofVehicle,
    environment: SixDofEnvironment,
    dt_s: float,
) -> SixDofState:
    k1 = derivatives(state, actuation, vehicle, environment)
    k2 = derivatives(
        add_scaled(state, k1, 0.5 * dt_s),
        actuation,
        vehicle,
        environment,
    )
    k3 = derivatives(
        add_scaled(state, k2, 0.5 * dt_s),
        actuation,
        vehicle,
        environment,
    )
    k4 = derivatives(
        add_scaled(state, k3, dt_s),
        actuation,
        vehicle,
        environment,
    )
    next_state = SixDofState(
        time_s=state.time_s + dt_s,
        position_inertial_m=state.position_inertial_m
        + dt_s
        / 6.0
        * (
            k1.position_rate_mps
            + 2.0 * k2.position_rate_mps
            + 2.0 * k3.position_rate_mps
            + k4.position_rate_mps
        ),
        velocity_inertial_mps=state.velocity_inertial_mps
        + dt_s
        / 6.0
        * (
            k1.velocity_rate_mps2
            + 2.0 * k2.velocity_rate_mps2
            + 2.0 * k3.velocity_rate_mps2
            + k4.velocity_rate_mps2
        ),
        quaternion_body_to_inertial=quaternion_normalize(
            state.quaternion_body_to_inertial
            + dt_s
            / 6.0
            * (
                k1.quaternion_rate
                + 2.0 * k2.quaternion_rate
                + 2.0 * k3.quaternion_rate
                + k4.quaternion_rate
            )
        ),
        angular_velocity_body_radps=state.angular_velocity_body_radps
        + dt_s
        / 6.0
        * (
            k1.angular_acceleration_body_radps2
            + 2.0 * k2.angular_acceleration_body_radps2
            + 2.0 * k3.angular_acceleration_body_radps2
            + k4.angular_acceleration_body_radps2
        ),
        mass_kg=max(
            vehicle.dry_mass_kg,
            state.mass_kg
            + dt_s
            / 6.0
            * (
                k1.mass_rate_kgps
                + 2.0 * k2.mass_rate_kgps
                + 2.0 * k3.mass_rate_kgps
                + k4.mass_rate_kgps
            ),
        ),
    )
    return next_state
