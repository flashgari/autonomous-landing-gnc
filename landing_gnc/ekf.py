"""Planar inertial sensor emulation and an eight-state error-state EKF.

The nominal state is [x, z, vx, vz, theta, b_ax, b_az, b_g]. Accelerometer
biases are expressed in the body frame. The covariance propagates a local
linear error state about the nonlinear strapdown inertial trajectory.
"""

import math
import random
from dataclasses import dataclass

import numpy as np

from .models import EkfConfig, EkfSensorModel, Environment, FaultScenario, State
from .navigation import wrap_angle


@dataclass(frozen=True)
class ImuMeasurement:
    time_s: float
    specific_force_body_x_mps2: float
    specific_force_body_z_mps2: float
    gyro_radps: float


@dataclass(frozen=True)
class GpsMeasurement:
    time_s: float
    x_m: float
    z_m: float
    vx_mps: float
    vz_mps: float


@dataclass(frozen=True)
class RadarMeasurement:
    time_s: float
    altitude_m: float


@dataclass(frozen=True)
class AttitudeMeasurement:
    time_s: float
    theta_rad: float


class EkfSensorSuite:
    """Generate IMU, GPS, radar-altimeter, and attitude-aiding measurements."""

    def __init__(
        self,
        model: EkfSensorModel,
        rng: random.Random,
        environment: Environment,
        fault: FaultScenario | None = None,
    ) -> None:
        self.model = model
        self.rng = rng
        self.environment = environment
        self.fault = fault or FaultScenario()
        self.accel_bias_x_mps2 = rng.gauss(0.0, model.accel_initial_bias_sigma_mps2)
        self.accel_bias_z_mps2 = rng.gauss(0.0, model.accel_initial_bias_sigma_mps2)
        self.gyro_bias_radps = rng.gauss(0.0, model.gyro_initial_bias_sigma_radps)

    @property
    def true_bias_vector(self) -> np.ndarray:
        return np.array(
            [
                self.accel_bias_x_mps2,
                self.accel_bias_z_mps2,
                self.gyro_bias_radps,
            ],
            dtype=float,
        )

    def measure_imu(
        self,
        state: State,
        inertial_ax_mps2: float,
        inertial_az_mps2: float,
        dt_s: float,
    ) -> ImuMeasurement:
        dt = max(0.0, dt_s)
        if dt > 0.0:
            self.accel_bias_x_mps2 += self.rng.gauss(
                0.0,
                self.model.accel_bias_random_walk_mps2_sqrt_s * math.sqrt(dt),
            )
            self.accel_bias_z_mps2 += self.rng.gauss(
                0.0,
                self.model.accel_bias_random_walk_mps2_sqrt_s * math.sqrt(dt),
            )
            self.gyro_bias_radps += self.rng.gauss(
                0.0,
                self.model.gyro_bias_random_walk_radps_sqrt_s * math.sqrt(dt),
            )

        noise_dt = max(dt, 0.02)
        accel_sample_sigma = self.model.accel_noise_density_mps2_sqrt_hz / math.sqrt(noise_dt)
        gyro_sample_sigma = self.model.gyro_noise_density_radps_sqrt_hz / math.sqrt(noise_dt)

        # An accelerometer measures specific force, not gravitational acceleration.
        specific_inertial_x = inertial_ax_mps2
        specific_inertial_z = inertial_az_mps2 + self.environment.gravity_mps2
        c = math.cos(state.theta_rad)
        s = math.sin(state.theta_rad)
        specific_body_x = c * specific_inertial_x - s * specific_inertial_z
        specific_body_z = s * specific_inertial_x + c * specific_inertial_z

        return ImuMeasurement(
            time_s=state.time_s,
            specific_force_body_x_mps2=specific_body_x
            + self.accel_bias_x_mps2
            + self.rng.gauss(0.0, accel_sample_sigma),
            specific_force_body_z_mps2=specific_body_z
            + self.accel_bias_z_mps2
            + self.rng.gauss(0.0, accel_sample_sigma),
            gyro_radps=state.omega_radps
            + self.gyro_bias_radps
            + self.rng.gauss(0.0, gyro_sample_sigma),
        )

    def gps_available(self, time_s: float) -> bool:
        start = self.fault.gps_dropout_start_s
        end = self.fault.gps_dropout_end_s
        if start is None:
            return True
        if time_s < start:
            return True
        return end is not None and time_s >= end

    def measure_gps(self, state: State) -> GpsMeasurement | None:
        if not self.gps_available(state.time_s):
            return None
        return GpsMeasurement(
            time_s=state.time_s,
            x_m=state.x_m + self.rng.gauss(0.0, self.model.gps_position_sigma_m),
            z_m=state.z_m + self.rng.gauss(0.0, self.model.gps_position_sigma_m),
            vx_mps=state.vx_mps + self.rng.gauss(0.0, self.model.gps_velocity_sigma_mps),
            vz_mps=state.vz_mps + self.rng.gauss(0.0, self.model.gps_velocity_sigma_mps),
        )

    def measure_radar(self, state: State) -> RadarMeasurement:
        altitude_fault_m = 0.0
        if (
            self.fault.altitude_bias_step_time_s is not None
            and state.time_s >= self.fault.altitude_bias_step_time_s
        ):
            altitude_fault_m = self.fault.altitude_bias_step_m
        return RadarMeasurement(
            time_s=state.time_s,
            altitude_m=max(
                0.0,
                state.z_m
                + altitude_fault_m
                + self.rng.gauss(0.0, self.model.radar_altitude_sigma_m),
            ),
        )

    def measure_attitude(self, state: State) -> AttitudeMeasurement:
        return AttitudeMeasurement(
            time_s=state.time_s,
            theta_rad=state.theta_rad + self.rng.gauss(0.0, self.model.attitude_sigma_rad),
        )


class ErrorStateEkf:
    """Nonlinear strapdown propagation with linearized covariance correction."""

    STATE_SIZE = 8
    X, Z, VX, VZ, THETA, BAX, BAZ, BG = range(STATE_SIZE)

    def __init__(self, config: EkfConfig, gravity_mps2: float) -> None:
        self.config = config
        self.gravity_mps2 = gravity_mps2
        self.nominal: np.ndarray | None = None
        self.covariance: np.ndarray | None = None
        self.time_s: float | None = None
        self.latest_corrected_rate_radps = 0.0
        self.latest_nis = {"gps": math.nan, "radar": math.nan, "attitude": math.nan}
        self.nis_history: dict[str, list[float]] = {
            "gps": [],
            "radar": [],
            "attitude": [],
        }
        self.accepted_updates = {"gps": 0, "radar": 0, "attitude": 0}
        self.rejected_updates = {"gps": 0, "radar": 0, "attitude": 0}

    @property
    def initialized(self) -> bool:
        return self.nominal is not None and self.covariance is not None

    @property
    def rejection_count(self) -> int:
        return sum(self.rejected_updates.values())

    def initialize(
        self,
        gps: GpsMeasurement,
        attitude: AttitudeMeasurement,
        imu: ImuMeasurement,
    ) -> None:
        self.nominal = np.array(
            [
                gps.x_m,
                gps.z_m,
                gps.vx_mps,
                gps.vz_mps,
                attitude.theta_rad,
                0.0,
                0.0,
                0.0,
            ],
            dtype=float,
        )
        c = self.config
        initial_sigmas = np.array(
            [
                c.initial_position_sigma_m,
                c.initial_position_sigma_m,
                c.initial_velocity_sigma_mps,
                c.initial_velocity_sigma_mps,
                c.initial_attitude_sigma_rad,
                c.initial_accel_bias_sigma_mps2,
                c.initial_accel_bias_sigma_mps2,
                c.initial_gyro_bias_sigma_radps,
            ],
            dtype=float,
        )
        self.covariance = np.diag(initial_sigmas**2)
        self.time_s = gps.time_s
        self.latest_corrected_rate_radps = imu.gyro_radps

    def propagate(self, imu: ImuMeasurement, dt_s: float) -> None:
        if not self.initialized:
            raise RuntimeError("EKF must be initialized before propagation")
        assert self.nominal is not None
        assert self.covariance is not None
        dt = max(0.0, dt_s)
        if dt == 0.0:
            self.latest_corrected_rate_radps = imu.gyro_radps - self.nominal[self.BG]
            self.time_s = imu.time_s
            return

        x = self.nominal
        corrected_force = np.array(
            [
                imu.specific_force_body_x_mps2 - x[self.BAX],
                imu.specific_force_body_z_mps2 - x[self.BAZ],
            ],
            dtype=float,
        )
        corrected_rate = imu.gyro_radps - x[self.BG]
        theta_mid = x[self.THETA] + 0.5 * corrected_rate * dt
        c = math.cos(theta_mid)
        s = math.sin(theta_mid)
        rotation_body_to_inertial = np.array([[c, s], [-s, c]], dtype=float)
        specific_force_inertial = rotation_body_to_inertial @ corrected_force
        acceleration_inertial = specific_force_inertial + np.array(
            [0.0, -self.gravity_mps2],
            dtype=float,
        )

        x[self.X] += x[self.VX] * dt + 0.5 * acceleration_inertial[0] * dt * dt
        x[self.Z] += x[self.VZ] * dt + 0.5 * acceleration_inertial[1] * dt * dt
        x[self.VX] += acceleration_inertial[0] * dt
        x[self.VZ] += acceleration_inertial[1] * dt
        x[self.THETA] = wrap_angle(x[self.THETA] + corrected_rate * dt)
        self.latest_corrected_rate_radps = corrected_rate
        self.time_s = imu.time_s

        f = np.zeros((self.STATE_SIZE, self.STATE_SIZE), dtype=float)
        f[self.X, self.VX] = 1.0
        f[self.Z, self.VZ] = 1.0
        derivative_rotation = np.array([[-s, c], [-c, -s]], dtype=float)
        acceleration_attitude_sensitivity = derivative_rotation @ corrected_force
        f[self.VX, self.THETA] = acceleration_attitude_sensitivity[0]
        f[self.VZ, self.THETA] = acceleration_attitude_sensitivity[1]
        f[self.VX : self.VZ + 1, self.BAX : self.BAZ + 1] = -rotation_body_to_inertial
        f[self.THETA, self.BG] = -1.0

        phi = np.eye(self.STATE_SIZE) + f * dt + 0.5 * (f @ f) * dt * dt
        g = np.zeros((self.STATE_SIZE, 6), dtype=float)
        g[self.VX : self.VZ + 1, 0:2] = rotation_body_to_inertial
        g[self.THETA, 2] = 1.0
        g[self.BAX, 3] = 1.0
        g[self.BAZ, 4] = 1.0
        g[self.BG, 5] = 1.0
        q_density = np.diag(
            [
                self.config.accel_noise_density_mps2_sqrt_hz**2,
                self.config.accel_noise_density_mps2_sqrt_hz**2,
                self.config.gyro_noise_density_radps_sqrt_hz**2,
                self.config.accel_bias_random_walk_mps2_sqrt_s**2,
                self.config.accel_bias_random_walk_mps2_sqrt_s**2,
                self.config.gyro_bias_random_walk_radps_sqrt_s**2,
            ]
        )
        process_covariance = g @ q_density @ g.T * dt
        self.covariance = phi @ self.covariance @ phi.T + process_covariance
        self._symmetrize_covariance()

    def update_gps(self, measurement: GpsMeasurement) -> bool:
        h = np.zeros((4, self.STATE_SIZE), dtype=float)
        h[0, self.X] = 1.0
        h[1, self.Z] = 1.0
        h[2, self.VX] = 1.0
        h[3, self.VZ] = 1.0
        values = np.array(
            [measurement.x_m, measurement.z_m, measurement.vx_mps, measurement.vz_mps],
            dtype=float,
        )
        r = np.diag(
            [
                self.config.gps_position_sigma_m**2,
                self.config.gps_position_sigma_m**2,
                self.config.gps_velocity_sigma_mps**2,
                self.config.gps_velocity_sigma_mps**2,
            ]
        )
        return self._measurement_update("gps", values, h, r, self.config.gps_nis_gate)

    def update_radar(self, measurement: RadarMeasurement) -> bool:
        h = np.zeros((1, self.STATE_SIZE), dtype=float)
        h[0, self.Z] = 1.0
        return self._measurement_update(
            "radar",
            np.array([measurement.altitude_m], dtype=float),
            h,
            np.array([[self.config.radar_altitude_sigma_m**2]], dtype=float),
            self.config.radar_nis_gate,
        )

    def update_attitude(self, measurement: AttitudeMeasurement) -> bool:
        h = np.zeros((1, self.STATE_SIZE), dtype=float)
        h[0, self.THETA] = 1.0
        return self._measurement_update(
            "attitude",
            np.array([measurement.theta_rad], dtype=float),
            h,
            np.array([[self.config.attitude_sigma_rad**2]], dtype=float),
            self.config.attitude_nis_gate,
            angle_measurement=True,
        )

    def _measurement_update(
        self,
        sensor_name: str,
        measurement: np.ndarray,
        h: np.ndarray,
        measurement_covariance: np.ndarray,
        nis_gate: float,
        angle_measurement: bool = False,
    ) -> bool:
        if not self.initialized:
            raise RuntimeError("EKF must be initialized before measurement updates")
        assert self.nominal is not None
        assert self.covariance is not None
        innovation = measurement - h @ self.nominal
        if angle_measurement:
            innovation[0] = wrap_angle(float(innovation[0]))
        innovation_covariance = h @ self.covariance @ h.T + measurement_covariance
        nis = float(innovation.T @ np.linalg.solve(innovation_covariance, innovation))
        self.latest_nis[sensor_name] = nis
        self.nis_history[sensor_name].append(nis)
        if nis > nis_gate:
            self.rejected_updates[sensor_name] += 1
            return False

        gain = np.linalg.solve(innovation_covariance, h @ self.covariance).T
        correction = gain @ innovation
        self.nominal += correction
        self.nominal[self.THETA] = wrap_angle(float(self.nominal[self.THETA]))
        identity = np.eye(self.STATE_SIZE)
        joseph_left = identity - gain @ h
        self.covariance = (
            joseph_left @ self.covariance @ joseph_left.T
            + gain @ measurement_covariance @ gain.T
        )
        self.accepted_updates[sensor_name] += 1
        self._symmetrize_covariance()
        return True

    def to_state(self, mass_kg: float) -> State:
        if not self.initialized:
            raise RuntimeError("EKF has not been initialized")
        assert self.nominal is not None
        return State(
            time_s=float(self.time_s or 0.0),
            x_m=float(self.nominal[self.X]),
            z_m=max(0.0, float(self.nominal[self.Z])),
            vx_mps=float(self.nominal[self.VX]),
            vz_mps=float(self.nominal[self.VZ]),
            theta_rad=float(self.nominal[self.THETA]),
            omega_radps=float(self.latest_corrected_rate_radps),
            mass_kg=mass_kg,
        )

    def consistency_diagnostics(
        self,
        truth: State,
        true_bias_vector: np.ndarray,
    ) -> dict[str, float]:
        if not self.initialized:
            return {}
        assert self.nominal is not None
        assert self.covariance is not None
        truth_vector = np.array(
            [
                truth.x_m,
                truth.z_m,
                truth.vx_mps,
                truth.vz_mps,
                truth.theta_rad,
                true_bias_vector[0],
                true_bias_vector[1],
                true_bias_vector[2],
            ],
            dtype=float,
        )
        error = self.nominal - truth_vector
        error[self.THETA] = wrap_angle(float(error[self.THETA]))
        nees = float(error.T @ np.linalg.solve(self.covariance, error))
        sigmas = np.sqrt(np.maximum(np.diag(self.covariance), 0.0))
        names = ("x", "z", "vx", "vz", "theta", "bax", "baz", "bg")
        diagnostics: dict[str, float] = {
            "ekf_nees": nees,
            "ekf_nis_gps": self.latest_nis["gps"],
            "ekf_nis_radar": self.latest_nis["radar"],
            "ekf_nis_attitude": self.latest_nis["attitude"],
            "ekf_estimated_bax_mps2": float(self.nominal[self.BAX]),
            "ekf_estimated_baz_mps2": float(self.nominal[self.BAZ]),
            "ekf_estimated_bg_radps": float(self.nominal[self.BG]),
            "ekf_true_bax_mps2": float(true_bias_vector[0]),
            "ekf_true_baz_mps2": float(true_bias_vector[1]),
            "ekf_true_bg_radps": float(true_bias_vector[2]),
        }
        for index, name in enumerate(names):
            diagnostics[f"ekf_sigma_{name}"] = float(sigmas[index])
            diagnostics[f"ekf_{name}_inside_3sigma"] = float(
                abs(error[index]) <= 3.0 * sigmas[index]
            )
        return diagnostics

    def summary(self) -> dict[str, float | int]:
        summary: dict[str, float | int] = {
            "ekf_rejected_updates": self.rejection_count,
        }
        measurement_dimensions = {"gps": 4.0, "radar": 1.0, "attitude": 1.0}
        for sensor_name, history in self.nis_history.items():
            summary[f"ekf_{sensor_name}_accepted_updates"] = self.accepted_updates[sensor_name]
            summary[f"ekf_{sensor_name}_rejected_updates"] = self.rejected_updates[sensor_name]
            if history:
                summary[f"ekf_mean_{sensor_name}_nis"] = sum(history) / len(history)
                summary[f"ekf_mean_{sensor_name}_normalized_nis"] = (
                    sum(history) / len(history) / measurement_dimensions[sensor_name]
                )
        return summary

    def _symmetrize_covariance(self) -> None:
        assert self.covariance is not None
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
