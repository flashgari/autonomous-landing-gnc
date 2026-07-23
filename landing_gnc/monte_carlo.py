"""Monte Carlo dispersion campaign for powered landing verification."""

import csv
import random
from dataclasses import replace
from pathlib import Path

from .models import Environment, State, Vehicle
from .sim import default_initial_state, run_simulation, write_json


def sample_case(rng: random.Random, case_id: int) -> tuple[Vehicle, Environment, State, dict]:
    base_vehicle = Vehicle()
    base_env = Environment()
    nominal = default_initial_state(base_vehicle)

    thrust_scale = rng.uniform(0.97, 1.03)
    propellant_scale = rng.uniform(0.94, 1.04)
    drag_scale = rng.uniform(0.85, 1.20)
    density_scale = rng.uniform(0.85, 1.15)
    wind_x = rng.uniform(-14.0, 18.0)

    vehicle = replace(
        base_vehicle,
        propellant_mass_kg=base_vehicle.propellant_mass_kg * propellant_scale,
        max_thrust_n=base_vehicle.max_thrust_n * thrust_scale,
        drag_coefficient=base_vehicle.drag_coefficient * drag_scale,
    )
    env = replace(
        base_env,
        air_density_kg_m3=base_env.air_density_kg_m3 * density_scale,
        wind_x_mps=wind_x,
    )
    initial = replace(
        nominal,
        x_m=nominal.x_m + rng.gauss(0.0, 8.0),
        vx_mps=nominal.vx_mps + rng.gauss(0.0, 1.4),
        vz_mps=nominal.vz_mps + rng.gauss(0.0, 2.2),
        theta_rad=nominal.theta_rad + rng.gauss(0.0, 0.012),
        mass_kg=vehicle.wet_mass_kg,
    )

    dispersion = {
        "case_id": case_id,
        "thrust_scale": thrust_scale,
        "propellant_scale": propellant_scale,
        "drag_scale": drag_scale,
        "density_scale": density_scale,
        "wind_x_mps": wind_x,
        "initial_x_m": initial.x_m,
        "initial_vx_mps": initial.vx_mps,
        "initial_vz_mps": initial.vz_mps,
        "initial_theta_deg": initial.theta_rad * 180.0 / 3.141592653589793,
    }
    return vehicle, env, initial, dispersion


def classify_failure(metrics: dict) -> str:
    if metrics["success"]:
        return "success"
    if metrics["final_altitude_m"] > 0.05:
        return "time_horizon"
    if metrics["propellant_remaining_kg"] <= 1.0:
        return "propellant_depletion"
    if abs(metrics["touchdown_vz_mps"]) >= 2.5:
        return "vertical_speed"
    if abs(metrics["landing_x_error_m"]) >= 3.0:
        return "pad_miss"
    if abs(metrics["touchdown_vx_mps"]) >= 1.0:
        return "lateral_speed"
    if metrics["max_abs_tilt_deg"] >= 12.0:
        return "tilt_limit"
    return "combined_margin"


def run_monte_carlo(
    num_cases=200,
    seed=4242,
    dt_s=0.08,
    guidance_mode="baseline",
    duration_s=70.0,
    navigation_mode="truth",
    actuator_mode="ideal",
):
    rng = random.Random(seed)
    rows = []
    for case_id in range(num_cases):
        vehicle, env, initial, dispersion = sample_case(rng, case_id)
        _, metrics, _ = run_simulation(
            vehicle=vehicle,
            env=env,
            initial_state=initial,
            dt_s=dt_s,
            duration_s=duration_s,
            guidance_mode=guidance_mode,
            navigation_mode=navigation_mode,
            actuator_mode=actuator_mode,
            rng_seed=seed + 100_000 + case_id,
        )
        row = {
            **dispersion,
            **metrics,
            "failure_mode": classify_failure(metrics),
            "guidance_mode": guidance_mode,
            "navigation_mode": navigation_mode,
            "actuator_mode": actuator_mode,
        }
        rows.append(row)
    return rows, summarize(rows, seed, guidance_mode, navigation_mode, actuator_mode)


def summarize(
    rows: list[dict],
    seed: int,
    guidance_mode: str,
    navigation_mode: str = "truth",
    actuator_mode: str = "ideal",
) -> dict:
    n = len(rows)
    successes = [r for r in rows if r["success"]]
    failure_modes: dict[str, int] = {}
    for row in rows:
        failure_modes[row["failure_mode"]] = failure_modes.get(row["failure_mode"], 0) + 1

    def percentile(key, p):
        values = sorted(float(r[key]) for r in rows)
        idx = min(len(values) - 1, max(0, round((len(values) - 1) * p)))
        return values[idx]

    return {
        "seed": seed,
        "guidance_mode": guidance_mode,
        "navigation_mode": navigation_mode,
        "actuator_mode": actuator_mode,
        "num_cases": n,
        "successes": len(successes),
        "success_rate": len(successes) / n if n else 0.0,
        "failure_modes": failure_modes,
        "p50_abs_landing_error_m": percentile_abs(rows, "landing_x_error_m", 0.50),
        "p95_abs_landing_error_m": percentile_abs(rows, "landing_x_error_m", 0.95),
        "p50_touchdown_speed_mps": percentile("touchdown_speed_mps", 0.50),
        "p95_touchdown_speed_mps": percentile("touchdown_speed_mps", 0.95),
        "min_propellant_remaining_kg": min(r["propellant_remaining_kg"] for r in rows),
        "max_abs_tilt_deg": max(r["max_abs_tilt_deg"] for r in rows),
        "max_abs_gimbal_deg": max(r["max_abs_gimbal_deg"] for r in rows),
        "max_abs_throttle_tracking_error": max(r["max_abs_throttle_tracking_error"] for r in rows),
    }


def percentile_abs(rows: list[dict], key: str, p: float) -> float:
    values = sorted(abs(float(r[key])) for r in rows)
    idx = min(len(values) - 1, max(0, round((len(values) - 1) * p)))
    return values[idx]


def write_monte_carlo_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_monte_carlo_outputs(rows: list[dict], summary: dict, outdir: Path = Path("outputs")) -> None:
    suffix = "" if summary["guidance_mode"] == "baseline" else f'_{summary["guidance_mode"]}'
    write_monte_carlo_csv(rows, outdir / f"monte_carlo_landing{suffix}.csv")
    write_json(summary, outdir / f"monte_carlo_summary{suffix}.json")


def compare_summaries(summaries: list[dict]) -> dict:
    by_mode = {summary["guidance_mode"]: summary for summary in summaries}
    comparison = {
        "modes": list(by_mode),
        "summaries": by_mode,
    }
    if "baseline" in by_mode and "corridor" in by_mode:
        baseline = by_mode["baseline"]
        corridor = by_mode["corridor"]
        comparison["delta"] = {
            "success_rate_points": 100.0 * (corridor["success_rate"] - baseline["success_rate"]),
            "p95_abs_landing_error_m": corridor["p95_abs_landing_error_m"] - baseline["p95_abs_landing_error_m"],
            "p95_touchdown_speed_mps": corridor["p95_touchdown_speed_mps"] - baseline["p95_touchdown_speed_mps"],
            "max_abs_tilt_deg": corridor["max_abs_tilt_deg"] - baseline["max_abs_tilt_deg"],
            "max_abs_gimbal_deg": corridor["max_abs_gimbal_deg"] - baseline["max_abs_gimbal_deg"],
        }
    return comparison
