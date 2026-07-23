"""Sensor emulation and an innovation-gated alpha-beta state estimator."""

import math
import random
from dataclasses import dataclass

from .dynamics import clamp
from .models import EstimatorConfig, FaultScenario, SensorModel, State


@dataclass(frozen=True)
class Measurement:
    time_s: float
    x_m: float
    z_m: float
    vx_mps: float
    vz_mps: float
    theta_rad: float
    omega_radps: float


class SensorSuite:
    """Generate noisy measurements with run-constant biases and optional faults."""

    def __init__(
        self,
        model: SensorModel,
        rng: random.Random,
        fault: FaultScenario | None = None,
    ) -> None:
        self.model = model
        self.rng = rng
        self.fault = fault or FaultScenario()
        self.bias_x = rng.gauss(0.0, model.position_bias_sigma_m)
        self.bias_z = rng.gauss(0.0, model.altitude_bias_sigma_m)
        self.bias_vx = rng.gauss(0.0, model.velocity_bias_sigma_mps)
        self.bias_vz = rng.gauss(0.0, model.velocity_bias_sigma_mps)
        self.bias_theta = rng.gauss(0.0, model.attitude_bias_sigma_rad)
        self.bias_omega = rng.gauss(0.0, model.rate_bias_sigma_radps)

    def measure(self, state: State) -> Measurement:
        altitude_fault = 0.0
        if (
            self.fault.altitude_bias_step_time_s is not None
            and state.time_s >= self.fault.altitude_bias_step_time_s
        ):
            altitude_fault = self.fault.altitude_bias_step_m
        return Measurement(
            time_s=state.time_s,
            x_m=state.x_m + self.bias_x + self.rng.gauss(0.0, self.model.position_sigma_m),
            z_m=max(
                0.0,
                state.z_m
                + self.bias_z
                + altitude_fault
                + self.rng.gauss(0.0, self.model.altitude_sigma_m),
            ),
            vx_mps=state.vx_mps + self.bias_vx + self.rng.gauss(0.0, self.model.velocity_sigma_mps),
            vz_mps=state.vz_mps + self.bias_vz + self.rng.gauss(0.0, self.model.velocity_sigma_mps),
            theta_rad=state.theta_rad
            + self.bias_theta
            + self.rng.gauss(0.0, self.model.attitude_sigma_rad),
            omega_radps=state.omega_radps
            + self.bias_omega
            + self.rng.gauss(0.0, self.model.rate_sigma_radps),
        )


class AlphaBetaEstimator:
    """Constant-velocity predictor with gated measurement correction.

    The filter estimates the guidance states. Vehicle mass is supplied by the
    propulsion bookkeeping model because this first navigation layer does not
    attempt to infer propellant mass from accelerometer or engine telemetry.
    """

    def __init__(self, config: EstimatorConfig) -> None:
        self.config = config
        self.state: State | None = None
        self.last_measurement_time_s: float | None = None
        self.rejection_count = 0

    def initialize(self, measurement: Measurement, mass_kg: float) -> State:
        self.state = State(
            time_s=measurement.time_s,
            x_m=measurement.x_m,
            z_m=measurement.z_m,
            vx_mps=measurement.vx_mps,
            vz_mps=measurement.vz_mps,
            theta_rad=measurement.theta_rad,
            omega_radps=measurement.omega_radps,
            mass_kg=mass_kg,
        )
        self.last_measurement_time_s = measurement.time_s
        return self.state

    def predict(self, time_s: float, mass_kg: float) -> State:
        if self.state is None:
            raise RuntimeError("estimator must be initialized before prediction")
        dt = max(0.0, time_s - self.state.time_s)
        self.state = State(
            time_s=time_s,
            x_m=self.state.x_m + self.state.vx_mps * dt,
            z_m=max(0.0, self.state.z_m + self.state.vz_mps * dt),
            vx_mps=self.state.vx_mps,
            vz_mps=self.state.vz_mps,
            theta_rad=self.state.theta_rad + self.state.omega_radps * dt,
            omega_radps=self.state.omega_radps,
            mass_kg=mass_kg,
        )
        return self.state

    def update(self, measurement: Measurement, mass_kg: float) -> State:
        if self.state is None:
            return self.initialize(measurement, mass_kg)

        previous_time = (
            measurement.time_s
            if self.last_measurement_time_s is None
            else self.last_measurement_time_s
        )
        dt = max(1.0e-3, measurement.time_s - previous_time)
        predicted = self.predict(measurement.time_s, mass_kg)
        c = self.config

        x_residual, x_rejected = gate_innovation(
            measurement.x_m - predicted.x_m,
            c.horizontal_innovation_limit_m,
        )
        z_residual, z_rejected = gate_innovation(
            measurement.z_m - predicted.z_m,
            c.altitude_innovation_limit_m,
        )
        theta_residual, theta_rejected = gate_innovation(
            wrap_angle(measurement.theta_rad - predicted.theta_rad),
            c.attitude_innovation_limit_rad,
        )
        self.rejection_count += int(x_rejected) + int(z_rejected) + int(theta_rejected)

        vx_from_position = predicted.vx_mps + c.position_beta * x_residual / dt
        vz_from_position = predicted.vz_mps + c.position_beta * z_residual / dt
        omega_from_attitude = predicted.omega_radps + c.attitude_beta * theta_residual / dt

        self.state = State(
            time_s=measurement.time_s,
            x_m=predicted.x_m + c.position_alpha * x_residual,
            z_m=max(0.0, predicted.z_m + c.position_alpha * z_residual),
            vx_mps=blend(vx_from_position, measurement.vx_mps, c.velocity_measurement_blend),
            vz_mps=blend(vz_from_position, measurement.vz_mps, c.velocity_measurement_blend),
            theta_rad=predicted.theta_rad + c.attitude_alpha * theta_residual,
            omega_radps=blend(omega_from_attitude, measurement.omega_radps, c.rate_measurement_blend),
            mass_kg=mass_kg,
        )
        self.last_measurement_time_s = measurement.time_s
        return self.state


def blend(predicted: float, measured: float, measurement_weight: float) -> float:
    return (1.0 - measurement_weight) * predicted + measurement_weight * measured


def gate_innovation(residual: float, limit: float) -> tuple[float, bool]:
    if abs(residual) > limit:
        return 0.0, True
    return clamp(residual, -limit, limit), False


def wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi
