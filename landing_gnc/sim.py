"""Simulation entry points and CSV/metric helpers."""

import csv
import json
import math
from dataclasses import asdict
from pathlib import Path

from .dynamics import rk4_step
from .guidance import corridor_guidance_command, guidance_command
from .models import AttitudeControl, Environment, Guidance, State, Vehicle


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
):
    vehicle = vehicle or Vehicle()
    env = env or Environment()
    guidance = guidance or Guidance()
    attitude = attitude or AttitudeControl()
    state = initial_state or default_initial_state(vehicle)
    rows = []

    n = int(duration_s / dt_s) + 1
    for _ in range(n):
        if guidance_mode == "baseline":
            cmd = guidance_command(state, vehicle, env, guidance, attitude)
        elif guidance_mode == "corridor":
            cmd = corridor_guidance_command(state, vehicle, env, guidance, attitude)
        else:
            raise ValueError(f"unknown guidance_mode: {guidance_mode}")
        rows.append(row_from_state_command(state, cmd, vehicle))
        if state.z_m <= 0.0 and state.vz_mps <= 0.0:
            break
        state = rk4_step(state, cmd, vehicle, env, dt_s)
        if state.z_m < 0.0:
            state.z_m = 0.0

    metrics = compute_metrics(rows, vehicle)
    config = {
        "vehicle": asdict(vehicle),
        "environment": asdict(env),
        "guidance": asdict(guidance),
        "attitude_control": asdict(attitude),
        "initial_state": asdict(initial_state or default_initial_state(vehicle)),
        "guidance_mode": guidance_mode,
    }
    return rows, metrics, config


def row_from_state_command(state: State, cmd, vehicle: Vehicle) -> dict:
    prop_remaining = max(0.0, state.mass_kg - vehicle.dry_mass_kg)
    return {
        "time_s": state.time_s,
        "x_m": state.x_m,
        "z_m": state.z_m,
        "vx_mps": state.vx_mps,
        "vz_mps": state.vz_mps,
        "theta_deg": math.degrees(state.theta_rad),
        "omega_deg_s": math.degrees(state.omega_radps),
        "mass_kg": state.mass_kg,
        "prop_remaining_kg": prop_remaining,
        "throttle": cmd.throttle,
        "gimbal_deg": math.degrees(cmd.gimbal_rad),
        "desired_theta_deg": math.degrees(cmd.desired_theta_rad),
        "desired_ax_mps2": cmd.desired_ax_mps2,
        "desired_az_mps2": cmd.desired_az_mps2,
    }


def compute_metrics(rows, vehicle: Vehicle) -> dict:
    final = rows[-1]
    max_tilt = max(abs(r["theta_deg"]) for r in rows)
    max_gimbal = max(abs(r["gimbal_deg"]) for r in rows)
    prop_used = rows[0]["prop_remaining_kg"] - final["prop_remaining_kg"]
    touchdown_speed = math.hypot(final["vx_mps"], final["vz_mps"])
    success = (
        final["z_m"] <= 0.05
        and abs(final["x_m"]) < 3.0
        and abs(final["vx_mps"]) < 1.0
        and abs(final["vz_mps"]) < 2.5
        and max_tilt < 12.0
        and final["mass_kg"] > vehicle.dry_mass_kg
    )
    return {
        "success": success,
        "final_time_s": final["time_s"],
        "final_altitude_m": final["z_m"],
        "landing_x_error_m": final["x_m"],
        "touchdown_vx_mps": final["vx_mps"],
        "touchdown_vz_mps": final["vz_mps"],
        "touchdown_speed_mps": touchdown_speed,
        "propellant_used_kg": prop_used,
        "propellant_remaining_kg": final["prop_remaining_kg"],
        "max_abs_tilt_deg": max_tilt,
        "max_abs_gimbal_deg": max_gimbal,
    }


def write_csv(rows, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_json(data, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2) + "\n")
