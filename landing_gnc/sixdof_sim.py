"""Closed-loop 6-DOF powered-descent simulation and evidence helpers."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np

from .sixdof_control import (
    SixDofActuatorStack,
    allocate_engine_commands,
    wrench_command,
)
from .sixdof_dynamics import derivatives, rk4_step
from .sixdof_math import (
    quaternion_from_euler,
    quaternion_to_euler,
    quaternion_to_matrix,
    tilt_from_vertical_rad,
)
from .sixdof_models import (
    SixDofActuatorModel,
    SixDofAttitudeControl,
    SixDofEnvironment,
    SixDofGuidance,
    SixDofScenario,
    SixDofState,
    SixDofVehicle,
)


def default_sixdof_initial_state(
    vehicle: SixDofVehicle,
) -> SixDofState:
    return SixDofState(
        time_s=0.0,
        position_inertial_m=np.array([18.0, -12.0, 720.0]),
        velocity_inertial_mps=np.array([-2.5, 1.5, -58.0]),
        quaternion_body_to_inertial=quaternion_from_euler(
            math.radians(0.6),
            math.radians(1.0),
            math.radians(2.0),
        ),
        angular_velocity_body_radps=np.radians(
            np.array([0.10, -0.08, 0.06])
        ),
        mass_kg=vehicle.wet_mass_kg,
    )


def run_sixdof_simulation(
    duration_s: float = 70.0,
    dt_s: float = 0.05,
    vehicle: SixDofVehicle | None = None,
    environment: SixDofEnvironment | None = None,
    guidance: SixDofGuidance | None = None,
    attitude_control: SixDofAttitudeControl | None = None,
    actuator_model: SixDofActuatorModel | None = None,
    scenario: SixDofScenario | None = None,
    initial_state: SixDofState | None = None,
    target_inertial_m: np.ndarray | None = None,
) -> tuple[list[dict], dict, dict]:
    vehicle = vehicle or SixDofVehicle()
    environment = environment or SixDofEnvironment()
    guidance = guidance or SixDofGuidance()
    attitude_control = (
        attitude_control or SixDofAttitudeControl()
    )
    actuator_model = actuator_model or SixDofActuatorModel()
    scenario = scenario or SixDofScenario()
    state = (
        initial_state.copy()
        if initial_state is not None
        else default_sixdof_initial_state(vehicle)
    )
    target = (
        np.asarray(target_inertial_m, dtype=float)
        if target_inertial_m is not None
        else np.zeros(3)
    )
    actuators = SixDofActuatorStack(
        actuator_model,
        vehicle,
        dt_s,
    )
    rows: list[dict] = []

    step_count = int(duration_s / dt_s) + 1
    for _ in range(step_count):
        requested_wrench = wrench_command(
            state,
            vehicle,
            environment,
            guidance,
            attitude_control,
            target,
        )
        allocated = allocate_engine_commands(
            state,
            requested_wrench,
            vehicle,
            failed_engine_index=(
                scenario.failed_engine_index
                if state.time_s >= scenario.engine_failure_time_s
                else None
            ),
        )
        achieved = actuators.step(
            allocated,
            state,
            requested_wrench,
            dt_s,
        )
        derivative = derivatives(
            state,
            achieved,
            vehicle,
            environment,
        )
        rows.append(
            row_from_sixdof(
                state,
                requested_wrench,
                allocated,
                achieved,
                derivative,
                vehicle,
                target,
            )
        )
        if (
            state.position_inertial_m[2] <= 0.0
            and state.velocity_inertial_mps[2] <= 0.0
        ):
            break
        state = rk4_step(
            state,
            achieved,
            vehicle,
            environment,
            dt_s,
        )
        if state.position_inertial_m[2] < 0.0:
            state.position_inertial_m[2] = 0.0

    metrics = compute_sixdof_metrics(
        rows,
        vehicle,
        target,
    )
    configuration = {
        "vehicle": _json_ready(asdict(vehicle)),
        "environment": _json_ready(asdict(environment)),
        "guidance": _json_ready(asdict(guidance)),
        "attitude_control": _json_ready(
            asdict(attitude_control)
        ),
        "actuator_model": _json_ready(asdict(actuator_model)),
        "scenario": _json_ready(asdict(scenario)),
        "initial_state": state_to_dict(
            initial_state
            if initial_state is not None
            else default_sixdof_initial_state(vehicle)
        ),
        "target_inertial_m": target.tolist(),
        "duration_s": duration_s,
        "dt_s": dt_s,
    }
    return rows, metrics, configuration


def row_from_sixdof(
    state,
    wrench,
    allocated,
    actuation,
    derivative,
    vehicle,
    target,
) -> dict:
    rotation = quaternion_to_matrix(
        state.quaternion_body_to_inertial
    )
    desired_rotation = quaternion_to_matrix(
        wrench.desired_quaternion_body_to_inertial
    )
    body_z = rotation[:, 2]
    desired_body_z = desired_rotation[:, 2]
    euler_angles = quaternion_to_euler(
        state.quaternion_body_to_inertial
    )
    horizontal_error = state.position_inertial_m[:2] - target[:2]
    desired_force_body = rotation.T @ wrench.force_inertial_n
    inertia = vehicle.inertia_diag_kg_m2(state.mass_kg)
    engine_arm = abs(
        vehicle.engine_positions_body_m(state.mass_kg)[0][2]
    )
    row = {
        "time_s": state.time_s,
        "x_m": state.position_inertial_m[0],
        "y_m": state.position_inertial_m[1],
        "z_m": state.position_inertial_m[2],
        "vx_mps": state.velocity_inertial_mps[0],
        "vy_mps": state.velocity_inertial_mps[1],
        "vz_mps": state.velocity_inertial_mps[2],
        "qw": state.quaternion_body_to_inertial[0],
        "qx": state.quaternion_body_to_inertial[1],
        "qy": state.quaternion_body_to_inertial[2],
        "qz": state.quaternion_body_to_inertial[3],
        "roll_deg": math.degrees(euler_angles[0]),
        "pitch_deg": math.degrees(euler_angles[1]),
        "yaw_deg": math.degrees(euler_angles[2]),
        "quaternion_norm": np.linalg.norm(
            state.quaternion_body_to_inertial
        ),
        "omega_x_deg_s": math.degrees(
            state.angular_velocity_body_radps[0]
        ),
        "omega_y_deg_s": math.degrees(
            state.angular_velocity_body_radps[1]
        ),
        "omega_z_deg_s": math.degrees(
            state.angular_velocity_body_radps[2]
        ),
        "tilt_deg": math.degrees(
            tilt_from_vertical_rad(
                state.quaternion_body_to_inertial
            )
        ),
        "desired_tilt_deg": math.degrees(
            math.acos(
                float(np.clip(desired_body_z[2], -1.0, 1.0))
            )
        ),
        "body_z_inertial_x": body_z[0],
        "body_z_inertial_y": body_z[1],
        "body_z_inertial_z": body_z[2],
        "mass_kg": state.mass_kg,
        "propellant_remaining_kg": max(
            0.0,
            state.mass_kg - vehicle.dry_mass_kg,
        ),
        "inertia_x_kg_m2": inertia[0],
        "inertia_y_kg_m2": inertia[1],
        "inertia_z_kg_m2": inertia[2],
        "engine_arm_m": engine_arm,
        "horizontal_error_m": np.linalg.norm(
            horizontal_error
        ),
        "target_x_m": target[0],
        "target_y_m": target[1],
        "desired_ax_mps2": wrench.desired_acceleration_inertial_mps2[
            0
        ],
        "desired_ay_mps2": wrench.desired_acceleration_inertial_mps2[
            1
        ],
        "desired_az_mps2": wrench.desired_acceleration_inertial_mps2[
            2
        ],
        "requested_force_body_x_n": desired_force_body[0],
        "requested_force_body_y_n": desired_force_body[1],
        "requested_force_body_z_n": desired_force_body[2],
        "achieved_force_body_x_n": actuation.force_body_n[0],
        "achieved_force_body_y_n": actuation.force_body_n[1],
        "achieved_force_body_z_n": actuation.force_body_n[2],
        "requested_torque_x_nm": wrench.torque_body_nm[0],
        "requested_torque_y_nm": wrench.torque_body_nm[1],
        "requested_torque_z_nm": wrench.torque_body_nm[2],
        "allocated_force_body_x_n": allocated.force_body_n[0],
        "allocated_force_body_y_n": allocated.force_body_n[1],
        "allocated_force_body_z_n": allocated.force_body_n[2],
        "allocated_torque_x_nm": allocated.torque_body_nm[0],
        "allocated_torque_y_nm": allocated.torque_body_nm[1],
        "allocated_torque_z_nm": allocated.torque_body_nm[2],
        "achieved_torque_x_nm": actuation.torque_body_nm[0],
        "achieved_torque_y_nm": actuation.torque_body_nm[1],
        "achieved_torque_z_nm": actuation.torque_body_nm[2],
        "force_residual_n": np.linalg.norm(
            actuation.force_residual_n
        ),
        "torque_residual_nm": np.linalg.norm(
            actuation.torque_residual_nm
        ),
        "allocation_residual": allocated.normalized_allocation_residual,
        "actuator_tracking_residual": (
            actuation.normalized_allocation_residual
        ),
        "gimbal_saturated_fraction": actuation.gimbal_saturated_fraction,
        "throttle_saturated_fraction": actuation.throttle_saturated_fraction,
        "total_thrust_n": actuation.total_thrust_n,
        "aerodynamic_force_n": np.linalg.norm(
            derivative.aerodynamic_force_body_n
        ),
        "aerodynamic_moment_nm": np.linalg.norm(
            derivative.aerodynamic_moment_body_nm
        ),
        "acceleration_x_mps2": derivative.velocity_rate_mps2[0],
        "acceleration_y_mps2": derivative.velocity_rate_mps2[1],
        "acceleration_z_mps2": derivative.velocity_rate_mps2[2],
    }
    for index, engine in enumerate(
        actuation.engine_commands,
        start=1,
    ):
        row[f"engine_{index}_throttle"] = engine.throttle
        row[f"engine_{index}_gimbal_x_deg"] = math.degrees(
            engine.gimbal_x_rad
        )
        row[f"engine_{index}_gimbal_y_deg"] = math.degrees(
            engine.gimbal_y_rad
        )
        row[f"engine_{index}_enabled"] = int(engine.enabled)
    return _json_ready(row)


def compute_sixdof_metrics(
    rows: list[dict],
    vehicle: SixDofVehicle,
    target: np.ndarray,
) -> dict:
    final = rows[-1]
    horizontal_error = math.hypot(
        final["x_m"] - target[0],
        final["y_m"] - target[1],
    )
    horizontal_speed = math.hypot(
        final["vx_mps"],
        final["vy_mps"],
    )
    touchdown_speed = math.sqrt(
        horizontal_speed**2 + final["vz_mps"] ** 2
    )
    maximum_tilt = max(row["tilt_deg"] for row in rows)
    maximum_angular_rate = max(
        math.sqrt(
            row["omega_x_deg_s"] ** 2
            + row["omega_y_deg_s"] ** 2
            + row["omega_z_deg_s"] ** 2
        )
        for row in rows
    )
    maximum_allocation_residual = max(
        row["allocation_residual"] for row in rows
    )
    p95_allocation_residual = percentile(
        [row["allocation_residual"] for row in rows],
        0.95,
    )
    tracking_rows = [
        row
        for row in rows
        if row["time_s"] >= 1.0
    ]
    success = (
        final["z_m"] <= 0.05
        and horizontal_error < 3.0
        and horizontal_speed < 1.0
        and abs(final["vz_mps"]) < 2.5
        and maximum_tilt < 12.0
        and final["mass_kg"] > vehicle.dry_mass_kg
    )
    return {
        "success": success,
        "final_time_s": final["time_s"],
        "landing_x_m": final["x_m"],
        "landing_y_m": final["y_m"],
        "horizontal_target_error_m": horizontal_error,
        "touchdown_horizontal_speed_mps": horizontal_speed,
        "touchdown_vertical_speed_mps": abs(final["vz_mps"]),
        "touchdown_speed_mps": touchdown_speed,
        "maximum_tilt_deg": maximum_tilt,
        "maximum_angular_rate_deg_s": maximum_angular_rate,
        "propellant_used_kg": rows[0][
            "propellant_remaining_kg"
        ]
        - final["propellant_remaining_kg"],
        "propellant_remaining_kg": final[
            "propellant_remaining_kg"
        ],
        "maximum_allocation_residual": maximum_allocation_residual,
        "p95_allocation_residual": p95_allocation_residual,
        "p95_actuator_tracking_residual_after_1s": percentile(
            [
                row["actuator_tracking_residual"]
                for row in tracking_rows
            ],
            0.95,
        ),
        "maximum_actuator_tracking_residual": max(
            row["actuator_tracking_residual"] for row in rows
        ),
        "maximum_force_residual_n": max(
            row["force_residual_n"] for row in rows
        ),
        "maximum_torque_residual_nm": max(
            row["torque_residual_nm"] for row in rows
        ),
        "maximum_gimbal_saturated_fraction": max(
            row["gimbal_saturated_fraction"] for row in rows
        ),
        "maximum_throttle_saturated_fraction": max(
            row["throttle_saturated_fraction"] for row in rows
        ),
        "maximum_quaternion_norm_error": max(
            abs(row["quaternion_norm"] - 1.0)
            for row in rows
        ),
    }


def state_to_dict(state: SixDofState) -> dict:
    return {
        "time_s": state.time_s,
        "position_inertial_m": state.position_inertial_m.tolist(),
        "velocity_inertial_mps": state.velocity_inertial_mps.tolist(),
        "quaternion_body_to_inertial": state.quaternion_body_to_inertial.tolist(),
        "angular_velocity_body_radps": state.angular_velocity_body_radps.tolist(),
        "mass_kg": state.mass_kg,
    }


def percentile(values: list[float], probability: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return (
        ordered[lower] * (1.0 - fraction)
        + ordered[upper] * fraction
    )


def write_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(rows[0].keys()),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)


def write_json(data: dict, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(_json_ready(data), indent=2, sort_keys=True)
        + "\n"
    )


def _json_ready(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.floating):
        return float(value)
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, dict):
        return {
            key: _json_ready(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    return value
