#!/usr/bin/env python3
"""Run matched deterministic and dispersed 6-DOF NMPC comparisons."""

from __future__ import annotations

import csv
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np

from landing_gnc.sixdof_math import quaternion_from_euler
from landing_gnc.sixdof_models import (
    SixDofEnvironment,
    SixDofNmpcConfig,
    SixDofScenario,
    SixDofVehicle,
)
from landing_gnc.sixdof_sim import (
    default_sixdof_initial_state,
    run_sixdof_simulation,
    write_csv,
    write_json,
)


SEED = 9242
CASE_COUNT = 24


def failure_mode(metrics: dict) -> str:
    if metrics["horizontal_target_error_m"] >= 3.0:
        return "footprint"
    if metrics["touchdown_horizontal_speed_mps"] >= 1.0:
        return "horizontal_speed"
    if metrics["touchdown_vertical_speed_mps"] >= 2.5:
        return "vertical_speed"
    if metrics["maximum_tilt_deg"] >= 12.0:
        return "attitude"
    if metrics["propellant_remaining_kg"] <= 0.0:
        return "propellant"
    return "success"


def sampled_case(
    rng: np.random.Generator,
    vehicle: SixDofVehicle,
):
    state = default_sixdof_initial_state(vehicle)
    state.position_inertial_m[:2] += rng.normal(0.0, 7.0, 2)
    state.velocity_inertial_mps[:2] += rng.normal(0.0, 1.0, 2)
    state.velocity_inertial_mps[2] += rng.normal(0.0, 2.5)
    state.mass_kg = float(
        np.clip(
            state.mass_kg + rng.normal(0.0, 200.0),
            vehicle.dry_mass_kg + 6_000.0,
            vehicle.wet_mass_kg,
        )
    )
    state.quaternion_body_to_inertial = quaternion_from_euler(
        math.radians(rng.normal(0.6, 1.0)),
        math.radians(rng.normal(1.0, 1.0)),
        math.radians(rng.normal(2.0, 2.0)),
    )
    state.angular_velocity_body_radps += np.radians(
        rng.normal(0.0, 0.12, 3)
    )
    wind = (
        float(rng.normal(10.0, 5.0)),
        float(rng.normal(-4.0, 4.0)),
        0.0,
    )
    return state, SixDofEnvironment(wind_inertial_mps=wind)


def summarize(rows: list[dict], mode: str) -> dict:
    subset = [row for row in rows if row["guidance_mode"] == mode]
    errors = np.array(
        [row["horizontal_target_error_m"] for row in subset]
    )
    propellant = np.array(
        [row["propellant_remaining_kg"] for row in subset]
    )
    vertical_speed = np.array(
        [row["touchdown_vertical_speed_mps"] for row in subset]
    )
    counts: dict[str, int] = {}
    for row in subset:
        counts[row["failure_mode"]] = (
            counts.get(row["failure_mode"], 0) + 1
        )
    summary = {
        "case_count": len(subset),
        "success_count": sum(row["success"] for row in subset),
        "success_rate": sum(row["success"] for row in subset) / len(subset),
        "median_horizontal_error_m": float(np.median(errors)),
        "p95_horizontal_error_m": float(np.quantile(errors, 0.95)),
        "median_propellant_remaining_kg": float(np.median(propellant)),
        "p95_touchdown_vertical_speed_mps": float(
            np.quantile(vertical_speed, 0.95)
        ),
        "failure_mode_counts": counts,
    }
    if mode == "nmpc":
        summary.update(
            {
                "mean_optimizer_solve_time_ms": float(
                    np.mean(
                        [
                            row["nmpc_mean_solve_time_ms"]
                            for row in subset
                        ]
                    )
                ),
                "maximum_optimizer_solve_time_ms": float(
                    np.max(
                        [
                            row["nmpc_maximum_solve_time_ms"]
                            for row in subset
                        ]
                    )
                ),
                "mean_optimizer_acceptance_rate": float(
                    np.mean(
                        [row["nmpc_acceptance_rate"] for row in subset]
                    )
                ),
            }
        )
    return summary


def main() -> None:
    vehicle = SixDofVehicle()
    nmpc_config = SixDofNmpcConfig()
    rng = np.random.default_rng(SEED)
    rows: list[dict] = []
    nmpc_solve_times_ms: list[float] = []
    for case_index in range(CASE_COUNT):
        initial_state, environment = sampled_case(rng, vehicle)
        for mode in ("baseline", "nmpc"):
            trajectory, metrics, _ = run_sixdof_simulation(
                duration_s=55.0,
                dt_s=0.05,
                vehicle=vehicle,
                environment=environment,
                initial_state=initial_state,
                guidance_mode=mode,
                nmpc_config=nmpc_config,
            )
            rows.append(
                {
                    "case_index": case_index,
                    "guidance_mode": mode,
                    "wind_x_mps": environment.wind_inertial_mps[0],
                    "wind_y_mps": environment.wind_inertial_mps[1],
                    "initial_x_m": initial_state.position_inertial_m[0],
                    "initial_y_m": initial_state.position_inertial_m[1],
                    "initial_vx_mps": initial_state.velocity_inertial_mps[0],
                    "initial_vy_mps": initial_state.velocity_inertial_mps[1],
                    "initial_vz_mps": initial_state.velocity_inertial_mps[2],
                    "initial_mass_kg": initial_state.mass_kg,
                    "success": int(metrics["success"]),
                    "failure_mode": failure_mode(metrics),
                    "horizontal_target_error_m": metrics[
                        "horizontal_target_error_m"
                    ],
                    "touchdown_horizontal_speed_mps": metrics[
                        "touchdown_horizontal_speed_mps"
                    ],
                    "touchdown_vertical_speed_mps": metrics[
                        "touchdown_vertical_speed_mps"
                    ],
                    "maximum_tilt_deg": metrics["maximum_tilt_deg"],
                    "propellant_remaining_kg": metrics[
                        "propellant_remaining_kg"
                    ],
                    "nmpc_acceptance_rate": metrics.get(
                        "nmpc_acceptance_rate",
                        0.0,
                    ),
                    "nmpc_mean_solve_time_ms": metrics.get(
                        "nmpc_mean_solve_time_ms",
                        0.0,
                    ),
                    "nmpc_maximum_solve_time_ms": max(
                        (
                            row.get("nmpc_solve_time_ms", 0.0)
                            for row in trajectory
                            if row.get("nmpc_replanned", 0.0) > 0.5
                        ),
                        default=0.0,
                    ),
                }
            )
            if mode == "nmpc":
                nmpc_solve_times_ms.extend(
                    row.get("nmpc_solve_time_ms", 0.0)
                    for row in trajectory
                    if row.get("nmpc_replanned", 0.0) > 0.5
                )
        print(f"completed matched case {case_index + 1}/{CASE_COUNT}")

    deterministic = {}
    deterministic_cases = {
        "calm": (
            SixDofEnvironment(wind_inertial_mps=(0.0, 0.0, 0.0)),
            SixDofScenario(name="calm"),
        ),
        "crosswind": (
            SixDofEnvironment(wind_inertial_mps=(12.0, -6.0, 0.0)),
            SixDofScenario(name="crosswind"),
        ),
        "high_crosswind": (
            SixDofEnvironment(wind_inertial_mps=(18.0, -10.0, 0.0)),
            SixDofScenario(name="high_crosswind"),
        ),
        "engine_out": (
            SixDofEnvironment(wind_inertial_mps=(8.0, -3.0, 0.0)),
            SixDofScenario(
                name="engine_out",
                failed_engine_index=0,
                engine_failure_time_s=12.0,
            ),
        ),
    }
    for case_name, (environment, scenario) in deterministic_cases.items():
        deterministic[case_name] = {}
        for mode in ("baseline", "nmpc"):
            trajectory, metrics, _ = run_sixdof_simulation(
                duration_s=55.0,
                dt_s=0.05,
                environment=environment,
                scenario=scenario,
                guidance_mode=mode,
                nmpc_config=nmpc_config,
            )
            write_csv(
                trajectory,
                Path(
                    f"outputs/sixdof_{mode}_{case_name}_comparison.csv"
                ),
            )
            deterministic[case_name][mode] = metrics

    # Solve-time maxima come from the deterministic trajectory telemetry and
    # are reported separately from the Monte Carlo outcome statistics.
    for case_name in deterministic_cases:
        path = Path(
            f"outputs/sixdof_nmpc_{case_name}_comparison.csv"
        )
        with path.open() as stream:
            nmpc_solve_times_ms.extend(
                float(row.get("nmpc_solve_time_ms", 0.0))
                for row in csv.DictReader(stream)
                if float(row.get("nmpc_replanned", 0.0)) > 0.5
            )

    write_csv(rows, Path("outputs/sixdof_nmpc_monte_carlo.csv"))
    summary = {
        "model": (
            "6-DOF nonlinear truth plant with reduced-order "
            "attitude-aware NMPC guidance"
        ),
        "seed": SEED,
        "matched_case_count_per_mode": CASE_COUNT,
        "nmpc_config": asdict(nmpc_config),
        "dispersion_definition": {
            "horizontal_position_sigma_m": 7.0,
            "horizontal_velocity_sigma_mps": 1.0,
            "vertical_velocity_sigma_mps": 2.5,
            "mass_sigma_kg": 200.0,
            "wind_x_mean_sigma_mps": [10.0, 5.0],
            "wind_y_mean_sigma_mps": [-4.0, 4.0],
        },
        "baseline": summarize(rows, "baseline"),
        "nmpc": summarize(rows, "nmpc"),
        "deterministic_cases": deterministic,
        "timing": {
            "replan_deadline_ms": 800.0,
            "observed_solve_count": len(nmpc_solve_times_ms),
            "observed_mean_solve_time_ms": (
                float(np.mean(nmpc_solve_times_ms))
                if nmpc_solve_times_ms
                else 0.0
            ),
            "observed_p95_solve_time_ms": (
                float(np.quantile(nmpc_solve_times_ms, 0.95))
                if nmpc_solve_times_ms
                else 0.0
            ),
            "observed_maximum_solve_time_ms": (
                float(np.max(nmpc_solve_times_ms))
                if nmpc_solve_times_ms
                else 0.0
            ),
        },
        "engine_out_interpretation": (
            "The fault supervisor invalidates the nominal NMPC plan, but "
            "the three-engine allocator remains rank-deficient for the "
            "requested six-axis wrench; this case is retained as a failure "
            "boundary rather than counted in the nominal dispersion campaign."
        ),
    }
    write_json(
        summary,
        Path("outputs/sixdof_nmpc_campaign_summary.json"),
    )
    print("Wrote 6-DOF NMPC campaign outputs")


if __name__ == "__main__":
    main()
