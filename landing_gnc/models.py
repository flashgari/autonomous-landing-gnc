"""Core data models for the powered-landing simulator."""

from dataclasses import dataclass


@dataclass(frozen=True)
class Vehicle:
    dry_mass_kg: float = 18_000.0
    propellant_mass_kg: float = 7_000.0
    max_thrust_n: float = 760_000.0
    min_throttle: float = 0.38
    max_gimbal_rad: float = 0.20
    isp_s: float = 282.0
    inertia_kg_m2: float = 1.7e6
    engine_moment_arm_m: float = 15.0
    rotational_damping_nms: float = 2.0e5
    reference_area_m2: float = 10.0
    drag_coefficient: float = 0.55

    @property
    def wet_mass_kg(self) -> float:
        return self.dry_mass_kg + self.propellant_mass_kg


@dataclass(frozen=True)
class Environment:
    gravity_mps2: float = 9.80665
    air_density_kg_m3: float = 0.65
    wind_x_mps: float = 8.0


@dataclass(frozen=True)
class Guidance:
    vertical_decel_mps2: float = 2.2
    terminal_descent_mps: float = 0.8
    lateral_kp: float = 0.030
    lateral_kd: float = 0.42
    vertical_kv: float = 2.1
    max_lateral_accel_mps2: float = 3.5
    max_vertical_accel_mps2: float = 28.0


@dataclass(frozen=True)
class AttitudeControl:
    kp: float = 0.55
    kd: float = 1.65


@dataclass
class State:
    time_s: float
    x_m: float
    z_m: float
    vx_mps: float
    vz_mps: float
    theta_rad: float
    omega_radps: float
    mass_kg: float


@dataclass(frozen=True)
class Command:
    throttle: float
    gimbal_rad: float
    desired_theta_rad: float
    desired_ax_mps2: float
    desired_az_mps2: float
