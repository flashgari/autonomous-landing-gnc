"""Simulation entry points and CSV/metric helpers."""

import csv
import json
import math
import random
from dataclasses import asdict
from pathlib import Path

from .actuators import ActuatorStack
from .dynamics import rk4_step
from .guidance import corridor_guidance_command, guidance_command
from .models import (
    ActuatorModel,
    AttitudeControl,
    Command,
    Environment,
    EstimatorConfig,
    FaultScenario,
    Guidance,
    SensorModel,
    State,
    Vehicle,
)
from .navigation import AlphaBetaEstimator, SensorSuite


def default_initial_state(vehicle: Vehicle) -> State:
    return State(
        time_s=0.0,
        x_m=18.0,
        z_m=720.0,
        vx_mps=-2.5,
        vz_mps=-58.0,
        theta_rad=math.radians(1.0),
        omega_radps=0.0,
        mass_kg=vehicle.wet_mass_kg,
    )


def run_simulation(
    duration_s=45.0,
    dt_s=0.05,
    vehicle: Vehicle | None = None,
    env: Environment | None = None,
    guidance: Guidance | None = None,
    attitude: AttitudeControl | None = None,
    initial_state: State | None = None,
    guidance_mode: str = "baseline",
    navigation_mode: str = "truth",
    actuator_mode: str = "ideal",
    sensor_model: SensorModel | None = None,
    estimator_config: EstimatorConfig | None = None,
    actuator_model: ActuatorModel | None = None,
    fault: FaultScenario | None = None,
    rng_seed: int = 0,
    target_x_m: float = 0.0,
):
    vehicle = vehicle or Vehicle()
    env = env or Environment()
    guidance = guidance or Guidance()
    attitude = attitude or AttitudeControl()
    sensor_model = sensor_model or SensorModel()
    estimator_config = estimator_config or EstimatorConfig()
    actuator_model = actuator_model or ActuatorModel()
    fault = fault or FaultScenario()
    state = initial_state or default_initial_state(vehicle)
    rows = []

    if navigation_mode not in ("truth", "estimated"):
        raise ValueError(f"unknown navigation_mode: {navigation_mode}")
    if actuator_mode not in ("ideal", "flight_like"):
        raise ValueError(f"unknown actuator_mode: {actuator_mode}")

    sensors = SensorSuite(sensor_model, random.Random(rng_seed), fault)
    estimator = AlphaBetaEstimator(estimator_config)
    estimated_state = None
    next_measurement_time_s = 0.0
    actuators = ActuatorStack(actuator_model, vehicle, dt_s)

    n = int(duration_s / dt_s) + 1
    for _ in range(n):
        if navigation_mode == "estimated":
            if state.time_s + 1.0e-9 >= next_measurement_time_s:
                measurement = sensors.measure(state)
                estimated_state = estimator.update(measurement, state.mass_kg)
                while next_measurement_time_s <= state.time_s + 1.0e-9:
                    next_measurement_time_s += sensor_model.sample_period_s
            else:
                estimated_state = estimator.predict(state.time_s, state.mass_kg)
            guidance_state = estimated_state
        else:
            guidance_state = state

        if guidance_mode == "baseline":
            commanded = guidance_command(
                guidance_state,
                vehicle,
                env,
                guidance,
                attitude,
                target_x_m=target_x_m,
            )
        elif guidance_mode == "corridor":
            commanded = corridor_guidance_command(
                guidance_state,
                vehicle,
                env,
                guidance,
                attitude,
                target_x_m=target_x_m,
            )
        else:
            raise ValueError(f"unknown guidance_mode: {guidance_mode}")

        applied = actuators.step(commanded, dt_s) if actuator_mode == "flight_like" else commanded
        thrust_scale = 1.0
        if fault.thrust_loss_time_s is not None and state.time_s >= fault.thrust_loss_time_s:
            thrust_scale = fault.thrust_scale_after_fault
            applied = Command(
                throttle=applied.throttle * thrust_scale,
                gimbal_rad=applied.gimbal_rad,
                desired_theta_rad=applied.desired_theta_rad,
                desired_ax_mps2=applied.desired_ax_mps2,
                desired_az_mps2=applied.desired_az_mps2,
            )
        rows.append(
            row_from_state_command(
                state,
                commanded,
                applied,
                vehicle,
                estimated_state,
                target_x_m,
                thrust_scale,
                estimator.rejection_count if navigation_mode == "estimated" else 0,
            )
        )
        if state.z_m <= 0.0 and state.vz_mps <= 0.0:
            break
        state = rk4_step(state, applied, vehicle, env, dt_s)
        if state.z_m < 0.0:
            state.z_m = 0.0

    metrics = compute_metrics(rows, vehicle, target_x_m)
    config = {
        "vehicle": asdict(vehicle),
        "environment": asdict(env),
        "guidance": asdict(guidance),
        "attitude_control": asdict(attitude),
        "sensor_model": asdict(sensor_model),
        "estimator": asdict(estimator_config),
        "actuator_model": asdict(actuator_model),
        "fault": asdict(fault),
        "initial_state": asdict(initial_state or default_initial_state(vehicle)),
        "guidance_mode": guidance_mode,
        "navigation_mode": navigation_mode,
        "actuator_mode": actuator_mode,
        "rng_seed": rng_seed,
        "target_x_m": target_x_m,
    }
    return rows, metrics, config


def row_from_state_command(
    state: State,
    commanded: Command,
    applied: Command,
    vehicle: Vehicle,
    estimate: State | None,
    target_x_m: float,
    thrust_scale: float,
    navigation_rejection_count: int,
) -> dict:
    prop_remaining = max(0.0, state.mass_kg - vehicle.dry_mass_kg)
    row = {
        "time_s": state.time_s,
        "x_m": state.x_m,
        "z_m": state.z_m,
        "vx_mps": state.vx_mps,
        "vz_mps": state.vz_mps,
        "theta_deg": math.degrees(state.theta_rad),
        "omega_deg_s": math.degrees(state.omega_radps),
        "mass_kg": state.mass_kg,
        "prop_remaining_kg": prop_remaining,
        "target_x_m": target_x_m,
        "target_error_m": state.x_m - target_x_m,
        "commanded_throttle": commanded.throttle,
        "commanded_gimbal_deg": math.degrees(commanded.gimbal_rad),
        "throttle": applied.throttle,
        "gimbal_deg": math.degrees(applied.gimbal_rad),
        "desired_theta_deg": math.degrees(commanded.desired_theta_rad),
        "desired_ax_mps2": commanded.desired_ax_mps2,
        "desired_az_mps2": commanded.desired_az_mps2,
        "thrust_scale": thrust_scale,
        "navigation_rejection_count": navigation_rejection_count,
    }
    if estimate is not None:
        row.update(
            {
                "estimated_x_m": estimate.x_m,
                "estimated_z_m": estimate.z_m,
                "estimated_vx_mps": estimate.vx_mps,
                "estimated_vz_mps": estimate.vz_mps,
                "estimated_theta_deg": math.degrees(estimate.theta_rad),
                "estimated_omega_deg_s": math.degrees(estimate.omega_radps),
                "x_estimation_error_m": estimate.x_m - state.x_m,
                "z_estimation_error_m": estimate.z_m - state.z_m,
                "vx_estimation_error_mps": estimate.vx_mps - state.vx_mps,
                "vz_estimation_error_mps": estimate.vz_mps - state.vz_mps,
                "theta_estimation_error_deg": math.degrees(estimate.theta_rad - state.theta_rad),
            }
        )
    return row


def compute_metrics(rows, vehicle: Vehicle, target_x_m: float = 0.0) -> dict:
    final = rows[-1]
    max_tilt = max(abs(r["theta_deg"]) for r in rows)
    max_gimbal = max(abs(r["gimbal_deg"]) for r in rows)
    prop_used = rows[0]["prop_remaining_kg"] - final["prop_remaining_kg"]
    touchdown_speed = math.hypot(final["vx_mps"], final["vz_mps"])
    target_error = final["x_m"] - target_x_m
    success = (
        final["z_m"] <= 0.05
        and abs(target_error) < 3.0
        and abs(final["vx_mps"]) < 1.0
        and abs(final["vz_mps"]) < 2.5
        and max_tilt < 12.0
        and final["mass_kg"] > vehicle.dry_mass_kg
    )
    metrics = {
        "success": success,
        "final_time_s": final["time_s"],
        "final_altitude_m": final["z_m"],
        "landing_x_m": final["x_m"],
        "target_x_m": target_x_m,
        "landing_x_error_m": target_error,
        "touchdown_vx_mps": final["vx_mps"],
        "touchdown_vz_mps": final["vz_mps"],
        "touchdown_speed_mps": touchdown_speed,
        "propellant_used_kg": prop_used,
        "propellant_remaining_kg": final["prop_remaining_kg"],
        "max_abs_tilt_deg": max_tilt,
        "max_abs_gimbal_deg": max_gimbal,
        "max_abs_commanded_gimbal_deg": max(abs(r["commanded_gimbal_deg"]) for r in rows),
        "max_abs_throttle_tracking_error": max(
            abs(r["commanded_throttle"] - r["throttle"]) for r in rows
        ),
        "navigation_rejection_count": int(final["navigation_rejection_count"]),
    }
    if "x_estimation_error_m" in final:
        metrics.update(
            {
                "rms_x_estimation_error_m": rms(rows, "x_estimation_error_m"),
                "rms_z_estimation_error_m": rms(rows, "z_estimation_error_m"),
                "rms_vx_estimation_error_mps": rms(rows, "vx_estimation_error_mps"),
                "rms_vz_estimation_error_mps": rms(rows, "vz_estimation_error_mps"),
                "rms_theta_estimation_error_deg": rms(rows, "theta_estimation_error_deg"),
            }
        )
    return metrics


def rms(rows, key: str) -> float:
    return math.sqrt(sum(float(row[key]) ** 2 for row in rows) / len(rows))


def write_csv(rows, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_json(data, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
