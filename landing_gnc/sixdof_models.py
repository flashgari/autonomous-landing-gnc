"""Data models for the 3D 6-DOF powered-descent milestone."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SixDofVehicle:
    dry_mass_kg: float = 18_000.0
    propellant_mass_kg: float = 7_000.0
    maximum_total_thrust_n: float = 760_000.0
    minimum_engine_throttle: float = 0.20
    maximum_gimbal_rad: float = 0.20
    specific_impulse_s: float = 282.0
    wet_inertia_diag_kg_m2: tuple[float, float, float] = (1.70e6, 1.70e6, 2.90e5)
    dry_inertia_diag_kg_m2: tuple[float, float, float] = (1.18e6, 1.18e6, 2.25e5)
    wet_engine_arm_m: float = 14.0
    dry_engine_arm_m: float = 15.5
    engine_cluster_radius_m: float = 1.8
    rotational_damping_nms: tuple[float, float, float] = (2.0e5, 2.0e5, 1.2e5)
    reference_area_m2: float = 10.0
    drag_coefficient: float = 0.55
    normal_force_slope: float = 1.8
    center_of_pressure_offset_m: float = 4.0

    @property
    def wet_mass_kg(self) -> float:
        return self.dry_mass_kg + self.propellant_mass_kg

    @property
    def maximum_engine_thrust_n(self) -> float:
        return self.maximum_total_thrust_n / 4.0

    def propellant_fraction(self, mass_kg: float) -> float:
        if self.propellant_mass_kg <= 0.0:
            return 0.0
        return float(
            np.clip(
                (mass_kg - self.dry_mass_kg) / self.propellant_mass_kg,
                0.0,
                1.0,
            )
        )

    def inertia_diag_kg_m2(self, mass_kg: float) -> np.ndarray:
        fraction = self.propellant_fraction(mass_kg)
        dry = np.asarray(self.dry_inertia_diag_kg_m2, dtype=float)
        wet = np.asarray(self.wet_inertia_diag_kg_m2, dtype=float)
        return dry + fraction * (wet - dry)

    def inertia_rate_diag_kg_m2_s(
        self,
        mass_rate_kgps: float,
    ) -> np.ndarray:
        if self.propellant_mass_kg <= 0.0:
            return np.zeros(3)
        dry = np.asarray(self.dry_inertia_diag_kg_m2, dtype=float)
        wet = np.asarray(self.wet_inertia_diag_kg_m2, dtype=float)
        derivative_with_mass = (
            wet - dry
        ) / self.propellant_mass_kg
        return derivative_with_mass * mass_rate_kgps

    def engine_positions_body_m(self, mass_kg: float) -> tuple[np.ndarray, ...]:
        fraction = self.propellant_fraction(mass_kg)
        arm = self.dry_engine_arm_m + fraction * (
            self.wet_engine_arm_m - self.dry_engine_arm_m
        )
        radius = self.engine_cluster_radius_m
        return (
            np.array([radius, radius, -arm]),
            np.array([-radius, radius, -arm]),
            np.array([-radius, -radius, -arm]),
            np.array([radius, -radius, -arm]),
        )


@dataclass(frozen=True)
class SixDofEnvironment:
    gravity_mps2: float = 9.80665
    air_density_kg_m3: float = 0.65
    wind_inertial_mps: tuple[float, float, float] = (8.0, -3.0, 0.0)


@dataclass(frozen=True)
class SixDofGuidance:
    vertical_deceleration_mps2: float = 2.2
    approach_gate_altitude_m: float = 90.0
    approach_descent_mps: float = 4.5
    flare_gate_altitude_m: float = 25.0
    flare_descent_mps: float = 2.2
    terminal_gate_altitude_m: float = 6.0
    terminal_descent_mps: float = 1.2
    horizontal_position_gain: float = 0.038
    horizontal_velocity_gain: float = 0.58
    corridor_gain: float = 0.070
    vertical_velocity_gain: float = 2.35
    maximum_horizontal_acceleration_mps2: float = 3.6
    maximum_vertical_acceleration_mps2: float = 28.0
    maximum_tilt_rad: float = 0.145
    yaw_reference_rad: float = 0.0


@dataclass(frozen=True)
class SixDofNmpcConfig:
    """Reduced-order nonlinear MPC configuration for the 6-DOF truth plant."""

    horizon_steps: int = 6
    replan_period_s: float = 0.80
    minimum_horizon_s: float = 5.0
    maximum_horizon_s: float = 28.0
    terminal_handoff_altitude_m: float = 25.0
    terminal_descent_mps: float = 1.0
    maximum_iterations: int = 3
    finite_difference_step_mps2: float = 0.035
    initial_trust_region_mps2: float = 1.8
    minimum_trust_region_mps2: float = 0.08
    maximum_trust_region_mps2: float = 4.0
    attitude_natural_frequency_radps: float = 1.55
    attitude_damping_ratio: float = 0.90
    position_weight: float = 0.025
    velocity_weight: float = 0.20
    terminal_position_weight: float = 1.8
    terminal_velocity_weight: float = 2.4
    attitude_lag_weight: float = 0.35
    body_rate_weight: float = 0.08
    thrust_weight: float = 0.0025
    thrust_slew_weight: float = 0.035
    corridor_weight: float = 1.2
    propellant_reserve_weight: float = 5.0
    regularization: float = 0.08
    acceptance_relative_improvement: float = 1.0e-5


@dataclass(frozen=True)
class SixDofAttitudeControl:
    proportional_gain_nm: tuple[float, float, float] = (5.0e6, 5.0e6, 1.8e6)
    derivative_gain_nms: tuple[float, float, float] = (4.0e6, 4.0e6, 1.4e6)
    maximum_torque_nm: tuple[float, float, float] = (2.0e6, 2.0e6, 3.5e5)


@dataclass(frozen=True)
class SixDofActuatorModel:
    throttle_time_constant_s: float = 0.22
    gimbal_time_constant_s: float = 0.10
    throttle_rate_limit_per_s: float = 1.8
    gimbal_rate_limit_radps: float = 0.50
    command_delay_s: float = 0.08


@dataclass
class SixDofState:
    time_s: float
    position_inertial_m: np.ndarray
    velocity_inertial_mps: np.ndarray
    quaternion_body_to_inertial: np.ndarray
    angular_velocity_body_radps: np.ndarray
    mass_kg: float

    def copy(self) -> "SixDofState":
        return SixDofState(
            time_s=self.time_s,
            position_inertial_m=self.position_inertial_m.copy(),
            velocity_inertial_mps=self.velocity_inertial_mps.copy(),
            quaternion_body_to_inertial=self.quaternion_body_to_inertial.copy(),
            angular_velocity_body_radps=self.angular_velocity_body_radps.copy(),
            mass_kg=self.mass_kg,
        )


@dataclass(frozen=True)
class EngineCommand:
    throttle: float
    gimbal_x_rad: float
    gimbal_y_rad: float
    enabled: bool = True


@dataclass(frozen=True)
class WrenchCommand:
    force_inertial_n: np.ndarray
    torque_body_nm: np.ndarray
    desired_quaternion_body_to_inertial: np.ndarray
    desired_acceleration_inertial_mps2: np.ndarray


@dataclass(frozen=True)
class AchievedActuation:
    engine_commands: tuple[EngineCommand, ...]
    force_body_n: np.ndarray
    torque_body_nm: np.ndarray
    total_thrust_n: float
    force_residual_n: np.ndarray
    torque_residual_nm: np.ndarray
    normalized_allocation_residual: float
    gimbal_saturated_fraction: float
    throttle_saturated_fraction: float


@dataclass(frozen=True)
class SixDofScenario:
    name: str = "nominal"
    failed_engine_index: int | None = None
    engine_failure_time_s: float = 0.0
