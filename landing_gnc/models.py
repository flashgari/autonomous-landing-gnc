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


@dataclass(frozen=True)
class SensorModel:
    """First-order sensor error model for the planar navigation suite."""

    sample_period_s: float = 0.10
    position_sigma_m: float = 0.55
    altitude_sigma_m: float = 0.40
    velocity_sigma_mps: float = 0.10
    attitude_sigma_rad: float = 0.0012
    rate_sigma_radps: float = 0.0015
    position_bias_sigma_m: float = 0.25
    altitude_bias_sigma_m: float = 0.22
    velocity_bias_sigma_mps: float = 0.035
    attitude_bias_sigma_rad: float = 0.0007
    rate_bias_sigma_radps: float = 0.0008


@dataclass(frozen=True)
class EstimatorConfig:
    """Alpha-beta gains and innovation gates for the navigation filter."""

    position_alpha: float = 0.62
    position_beta: float = 0.025
    velocity_measurement_blend: float = 0.62
    attitude_alpha: float = 0.72
    attitude_beta: float = 0.035
    rate_measurement_blend: float = 0.62
    horizontal_innovation_limit_m: float = 7.0
    altitude_innovation_limit_m: float = 5.0
    attitude_innovation_limit_rad: float = 0.045


@dataclass(frozen=True)
class ActuatorModel:
    """Command-path dynamics for throttle and thrust-vector control."""

    throttle_time_constant_s: float = 0.22
    gimbal_time_constant_s: float = 0.10
    throttle_rate_limit_per_s: float = 1.8
    gimbal_rate_limit_radps: float = 0.50
    throttle_deadband: float = 0.004
    gimbal_deadband_rad: float = 0.0007
    command_delay_s: float = 0.08


@dataclass(frozen=True)
class FaultScenario:
    """Injected off-nominal events used for deterministic fault testing."""

    name: str = "none"
    thrust_loss_time_s: float | None = None
    thrust_scale_after_fault: float = 1.0
    altitude_bias_step_time_s: float | None = None
    altitude_bias_step_m: float = 0.0


@dataclass(frozen=True)
class HazardZone:
    left_m: float
    right_m: float
    name: str = "hazard"


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
