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
class PredictiveGuidanceConfig:
    """Finite-horizon constrained acceleration-guidance configuration."""

    horizon_steps: int = 12
    replan_period_s: float = 0.60
    minimum_horizon_s: float = 2.0
    maximum_horizon_s: float = 30.0
    terminal_handoff_altitude_m: float = 160.0
    terminal_descent_mps: float = 0.75
    maximum_tilt_rad: float = 0.140
    glide_slope_ratio: float = 0.10
    glide_slope_base_m: float = 2.0
    maximum_ax_step_mps2: float = 2.2
    maximum_az_step_mps2: float = 8.0
    position_weight: float = 0.10
    velocity_weight: float = 0.35
    terminal_position_weight: float = 42.0
    terminal_velocity_weight: float = 34.0
    vertical_position_weight: float = 0.14
    vertical_velocity_weight: float = 0.45
    vertical_terminal_position_weight: float = 55.0
    vertical_terminal_velocity_weight: float = 48.0
    acceleration_weight: float = 0.035
    thrust_regularization_weight: float = 0.006
    slew_weight: float = 0.18
    admm_rho: float = 1.0
    admm_max_iterations: int = 90
    admm_absolute_tolerance: float = 0.025
    accepted_constraint_violation: float = 0.10


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
class EkfSensorModel:
    """Truth-side IMU and asynchronous aiding-sensor error model."""

    gps_sample_period_s: float = 0.20
    radar_sample_period_s: float = 0.10
    attitude_sample_period_s: float = 0.05
    gps_position_sigma_m: float = 0.75
    gps_velocity_sigma_mps: float = 0.12
    radar_altitude_sigma_m: float = 0.35
    attitude_sigma_rad: float = 0.0015
    accel_noise_density_mps2_sqrt_hz: float = 0.035
    gyro_noise_density_radps_sqrt_hz: float = 0.0007
    accel_initial_bias_sigma_mps2: float = 0.12
    gyro_initial_bias_sigma_radps: float = 0.0010
    accel_bias_random_walk_mps2_sqrt_s: float = 0.0020
    gyro_bias_random_walk_radps_sqrt_s: float = 0.00005


@dataclass(frozen=True)
class EkfConfig:
    """Error-state covariance, process-noise, and innovation-gate settings."""

    initial_position_sigma_m: float = 1.50
    initial_velocity_sigma_mps: float = 0.30
    initial_attitude_sigma_rad: float = 0.006
    initial_accel_bias_sigma_mps2: float = 0.20
    initial_gyro_bias_sigma_radps: float = 0.003
    accel_noise_density_mps2_sqrt_hz: float = 0.050
    gyro_noise_density_radps_sqrt_hz: float = 0.0010
    accel_bias_random_walk_mps2_sqrt_s: float = 0.0030
    gyro_bias_random_walk_radps_sqrt_s: float = 0.00008
    gps_position_sigma_m: float = 0.75
    gps_velocity_sigma_mps: float = 0.12
    radar_altitude_sigma_m: float = 0.35
    attitude_sigma_rad: float = 0.0015
    gps_nis_gate: float = 18.47
    radar_nis_gate: float = 9.0
    attitude_nis_gate: float = 9.0


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
    gps_dropout_start_s: float | None = None
    gps_dropout_end_s: float | None = None


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
