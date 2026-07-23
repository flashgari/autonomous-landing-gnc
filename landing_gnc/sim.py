"""Simulation entry points and CSV/metric helpers."""

import csv
import json
import math
import random
from dataclasses import asdict
from pathlib import Path

from .actuators import ActuatorStack
from .constrained_guidance import PredictiveGuidanceController
from .dynamics import derivatives, rk4_step
from .ekf import EkfSensorSuite, ErrorStateEkf
from .guidance import corridor_guidance_command, guidance_command
from .models import (
    ActuatorModel,
    AttitudeControl,
    Command,
    EkfConfig,
    EkfSensorModel,
    Environment,
    EstimatorConfig,
    FaultScenario,
    Guidance,
    PredictiveGuidanceConfig,
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
    ekf_sensor_model: EkfSensorModel | None = None,
    ekf_config: EkfConfig | None = None,
    predictive_guidance_config: PredictiveGuidanceConfig | None = None,
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
    ekf_sensor_model = ekf_sensor_model or EkfSensorModel()
    ekf_config = ekf_config or EkfConfig()
    predictive_guidance_config = (
        predictive_guidance_config or PredictiveGuidanceConfig()
    )
    actuator_model = actuator_model or ActuatorModel()
    fault = fault or FaultScenario()
    state = initial_state or default_initial_state(vehicle)
    rows = []

    if navigation_mode not in ("truth", "estimated", "ekf"):
        raise ValueError(f"unknown navigation_mode: {navigation_mode}")
    if actuator_mode not in ("ideal", "flight_like"):
        raise ValueError(f"unknown actuator_mode: {actuator_mode}")

    sensors = SensorSuite(sensor_model, random.Random(rng_seed), fault)
    estimator = AlphaBetaEstimator(estimator_config)
    ekf_sensors = EkfSensorSuite(
        ekf_sensor_model,
        random.Random(rng_seed + 31_337),
        env,
        fault,
    )
    ekf_estimator = ErrorStateEkf(ekf_config, env.gravity_mps2)
    predictive_controller = PredictiveGuidanceController(
        predictive_guidance_config
    )
    estimated_state = None
    ekf_diagnostics: dict[str, float] = {}
    optimizer_diagnostics: dict[str, float] = {}
    next_measurement_time_s = 0.0
    next_gps_time_s = 0.0
    next_radar_time_s = 0.0
    next_attitude_time_s = 0.0
    actuators = ActuatorStack(actuator_model, vehicle, dt_s)
    last_applied = Command(
        throttle=0.0,
        gimbal_rad=0.0,
        desired_theta_rad=state.theta_rad,
        desired_ax_mps2=0.0,
        desired_az_mps2=0.0,
    )

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
        elif navigation_mode == "ekf":
            derivative = derivatives(state, last_applied, vehicle, env)
            propagation_dt_s = (
                0.0
                if ekf_estimator.time_s is None
                else max(0.0, state.time_s - ekf_estimator.time_s)
            )
            imu = ekf_sensors.measure_imu(
                state,
                derivative.vx_mps,
                derivative.vz_mps,
                propagation_dt_s,
            )
            if not ekf_estimator.initialized:
                gps = ekf_sensors.measure_gps(state)
                if gps is None:
                    raise RuntimeError("GPS must be available for EKF initialization")
                attitude_measurement = ekf_sensors.measure_attitude(state)
                ekf_estimator.initialize(gps, attitude_measurement, imu)
                ekf_estimator.update_radar(ekf_sensors.measure_radar(state))
                next_gps_time_s = ekf_sensor_model.gps_sample_period_s
                next_radar_time_s = ekf_sensor_model.radar_sample_period_s
                next_attitude_time_s = ekf_sensor_model.attitude_sample_period_s
            else:
                ekf_estimator.propagate(imu, propagation_dt_s)
                if state.time_s + 1.0e-9 >= next_gps_time_s:
                    gps = ekf_sensors.measure_gps(state)
                    if gps is not None:
                        ekf_estimator.update_gps(gps)
                    while next_gps_time_s <= state.time_s + 1.0e-9:
                        next_gps_time_s += ekf_sensor_model.gps_sample_period_s
                if state.time_s + 1.0e-9 >= next_radar_time_s:
                    ekf_estimator.update_radar(ekf_sensors.measure_radar(state))
                    while next_radar_time_s <= state.time_s + 1.0e-9:
                        next_radar_time_s += ekf_sensor_model.radar_sample_period_s
                if state.time_s + 1.0e-9 >= next_attitude_time_s:
                    ekf_estimator.update_attitude(ekf_sensors.measure_attitude(state))
                    while next_attitude_time_s <= state.time_s + 1.0e-9:
                        next_attitude_time_s += ekf_sensor_model.attitude_sample_period_s
            estimated_state = ekf_estimator.to_state(state.mass_kg)
            ekf_diagnostics = ekf_estimator.consistency_diagnostics(
                state,
                ekf_sensors.true_bias_vector,
            )
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
        elif guidance_mode == "predictive":
            predictive_result = predictive_controller.command(
                guidance_state,
                vehicle,
                env,
                guidance,
                attitude,
                target_x_m=target_x_m,
            )
            commanded = predictive_result.command
            optimizer_diagnostics = predictive_result.diagnostics
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
                (
                    estimator.rejection_count
                    if navigation_mode == "estimated"
                    else ekf_estimator.rejection_count
                    if navigation_mode == "ekf"
                    else 0
                ),
                ekf_diagnostics,
                optimizer_diagnostics,
            )
        )
        if state.z_m <= 0.0 and state.vz_mps <= 0.0:
            break
        last_applied = applied
        state = rk4_step(state, applied, vehicle, env, dt_s)
        if state.z_m < 0.0:
            state.z_m = 0.0

    metrics = compute_metrics(rows, vehicle, target_x_m)
    if navigation_mode == "ekf":
        metrics.update(ekf_estimator.summary())
    if guidance_mode == "predictive":
        metrics.update(predictive_controller.summary())
    config = {
        "vehicle": asdict(vehicle),
        "environment": asdict(env),
        "guidance": asdict(guidance),
        "attitude_control": asdict(attitude),
        "sensor_model": asdict(sensor_model),
        "estimator": asdict(estimator_config),
        "ekf_sensor_model": asdict(ekf_sensor_model),
        "ekf": asdict(ekf_config),
        "predictive_guidance": asdict(predictive_guidance_config),
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
    ekf_diagnostics: dict[str, float] | None = None,
    optimizer_diagnostics: dict[str, float] | None = None,
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
                "theta_estimation_error_deg": math.degrees(
                    (estimate.theta_rad - state.theta_rad + math.pi)
                    % (2.0 * math.pi)
                    - math.pi
                ),
            }
        )
    if ekf_diagnostics:
        row.update(ekf_diagnostics)
    if optimizer_diagnostics:
        row.update(optimizer_diagnostics)
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
    if "ekf_nees" in final:
        metrics.update(
            {
                "ekf_mean_nees": finite_mean(rows, "ekf_nees"),
                "ekf_p95_nees": finite_percentile(rows, "ekf_nees", 0.95),
                "ekf_x_3sigma_coverage": finite_mean(rows, "ekf_x_inside_3sigma"),
                "ekf_z_3sigma_coverage": finite_mean(rows, "ekf_z_inside_3sigma"),
                "ekf_vx_3sigma_coverage": finite_mean(rows, "ekf_vx_inside_3sigma"),
                "ekf_vz_3sigma_coverage": finite_mean(rows, "ekf_vz_inside_3sigma"),
                "ekf_theta_3sigma_coverage": finite_mean(rows, "ekf_theta_inside_3sigma"),
                "ekf_bax_3sigma_coverage": finite_mean(rows, "ekf_bax_inside_3sigma"),
                "ekf_baz_3sigma_coverage": finite_mean(rows, "ekf_baz_inside_3sigma"),
                "ekf_bg_3sigma_coverage": finite_mean(rows, "ekf_bg_inside_3sigma"),
            }
        )
    if "optimizer_converged" in final:
        optimizer_rows = [
            row
            for row in rows
            if row["optimizer_replanned"] > 0.5
            and row["optimizer_terminal_handoff"] < 0.5
        ]
        metrics.update(
            {
                "optimizer_replan_count_from_rows": len(optimizer_rows),
                "optimizer_fallback_fraction": finite_mean(
                    optimizer_rows,
                    "optimizer_fallback",
                ),
                "optimizer_tilt_active_fraction": finite_mean(
                    optimizer_rows,
                    "optimizer_tilt_constraint_active",
                ),
                "optimizer_thrust_active_fraction": finite_mean(
                    optimizer_rows,
                    "optimizer_thrust_constraint_active",
                ),
                "optimizer_glideslope_active_fraction": finite_mean(
                    optimizer_rows,
                    "optimizer_glideslope_constraint_active",
                ),
                "optimizer_max_constraint_violation": max(
                    (
                        row["optimizer_max_constraint_violation"]
                        for row in optimizer_rows
                    ),
                    default=math.nan,
                ),
                "optimizer_min_tilt_margin_deg": min(
                    (
                        row["optimizer_minimum_tilt_margin_deg"]
                        for row in optimizer_rows
                    ),
                    default=math.nan,
                ),
                "optimizer_min_thrust_margin_mps2": min(
                    (
                        row["optimizer_minimum_thrust_margin_mps2"]
                        for row in optimizer_rows
                    ),
                    default=math.nan,
                ),
                "optimizer_min_glideslope_margin_m": min(
                    (
                        row["optimizer_minimum_glideslope_margin_m"]
                        for row in optimizer_rows
                    ),
                    default=math.nan,
                ),
            }
        )
    return metrics


def rms(rows, key: str) -> float:
    return math.sqrt(sum(float(row[key]) ** 2 for row in rows) / len(rows))


def finite_mean(rows, key: str) -> float:
    values = [float(row[key]) for row in rows if math.isfinite(float(row[key]))]
    return sum(values) / len(values) if values else math.nan


def finite_percentile(rows, key: str, p: float) -> float:
    values = sorted(float(row[key]) for row in rows if math.isfinite(float(row[key])))
    if not values:
        return math.nan
    index = min(len(values) - 1, max(0, round((len(values) - 1) * p)))
    return values[index]


def write_csv(rows, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_json(data, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
