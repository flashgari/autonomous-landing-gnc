#!/usr/bin/env python3
"""Run constrained predictive-guidance verification campaigns."""

import argparse
import csv
from dataclasses import replace
from pathlib import Path

from landing_gnc.models import FaultScenario
from landing_gnc.monte_carlo import run_monte_carlo, write_monte_carlo_csv
from landing_gnc.sim import (
    default_initial_state,
    run_simulation,
    write_csv,
    write_json,
)
from landing_gnc.models import Vehicle


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=200)
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--dt", type=float, default=0.08)
    parser.add_argument("--duration", type=float, default=70.0)
    args = parser.parse_args()

    deterministic = {}
    for guidance_mode in ("corridor", "predictive"):
        rows, metrics, config = run_simulation(
            duration_s=args.duration,
            dt_s=0.05,
            guidance_mode=guidance_mode,
            navigation_mode="ekf",
            actuator_mode="flight_like",
            rng_seed=args.seed,
        )
        write_csv(rows, Path(f"outputs/{guidance_mode}_ekf_nominal.csv"))
        write_json(config, Path(f"outputs/{guidance_mode}_ekf_nominal_config.json"))
        deterministic[guidance_mode] = metrics

    fault_rows, fault_metrics, fault_config = run_simulation(
        duration_s=args.duration,
        dt_s=0.05,
        guidance_mode="predictive",
        navigation_mode="ekf",
        actuator_mode="flight_like",
        rng_seed=args.seed,
        fault=FaultScenario(
            name="recoverable_thrust_loss",
            thrust_loss_time_s=8.0,
            thrust_scale_after_fault=0.92,
        ),
    )
    write_csv(fault_rows, Path("outputs/predictive_thrust_loss.csv"))
    write_json(fault_config, Path("outputs/predictive_thrust_loss_config.json"))
    deterministic["predictive_8_percent_thrust_loss"] = fault_metrics

    vehicle = Vehicle()
    large_divert_initial = replace(
        default_initial_state(vehicle),
        x_m=48.0,
        mass_kg=vehicle.wet_mass_kg,
    )
    divert_rows, divert_metrics, divert_config = run_simulation(
        duration_s=args.duration,
        dt_s=0.05,
        guidance_mode="predictive",
        navigation_mode="ekf",
        actuator_mode="flight_like",
        initial_state=large_divert_initial,
        rng_seed=args.seed,
    )
    write_csv(divert_rows, Path("outputs/predictive_48m_divert.csv"))
    write_json(
        divert_config,
        Path("outputs/predictive_48m_divert_config.json"),
    )
    deterministic["predictive_48m_divert"] = divert_metrics

    summaries = {}
    for guidance_mode in ("corridor", "predictive"):
        rows, summary = run_monte_carlo(
            num_cases=args.cases,
            seed=args.seed,
            dt_s=args.dt,
            duration_s=args.duration,
            guidance_mode=guidance_mode,
            navigation_mode="ekf",
            actuator_mode="flight_like",
        )
        write_monte_carlo_csv(
            rows,
            Path(f"outputs/monte_carlo_guidance_{guidance_mode}_ekf.csv"),
        )
        write_json(
            summary,
            Path(f"outputs/monte_carlo_guidance_{guidance_mode}_ekf_summary.json"),
        )
        summaries[guidance_mode] = summary

    reachability_rows = run_reachability_sweep(args.seed, args.duration)
    write_dict_csv(
        reachability_rows,
        Path("outputs/predictive_reachability_sweep.csv"),
    )

    corridor = summaries["corridor"]
    predictive = summaries["predictive"]
    comparison = {
        "campaign": {
            "cases_per_guidance_mode": args.cases,
            "seed": args.seed,
            "monte_carlo_dt_s": args.dt,
            "deterministic_dt_s": 0.05,
        },
        "deterministic_cases": deterministic,
        "monte_carlo_summaries": summaries,
        "delta_predictive_minus_corridor": {
            "success_rate_points": 100.0
            * (predictive["success_rate"] - corridor["success_rate"]),
            "p95_abs_landing_error_m": predictive["p95_abs_landing_error_m"]
            - corridor["p95_abs_landing_error_m"],
            "p95_touchdown_speed_mps": predictive["p95_touchdown_speed_mps"]
            - corridor["p95_touchdown_speed_mps"],
            "minimum_propellant_remaining_kg": (
                predictive["min_propellant_remaining_kg"]
                - corridor["min_propellant_remaining_kg"]
            ),
        },
        "reachability_sweep": reachability_rows,
    }
    write_json(comparison, Path("outputs/predictive_guidance_campaign.json"))
    print("Wrote constrained predictive-guidance campaign outputs")
    print(comparison)


def run_reachability_sweep(seed: int, duration_s: float) -> list[dict]:
    vehicle = Vehicle()
    nominal = default_initial_state(vehicle)
    rows = []
    for initial_x_m in range(0, 71, 10):
        for guidance_mode in ("corridor", "predictive"):
            initial = replace(
                nominal,
                x_m=float(initial_x_m),
                mass_kg=vehicle.wet_mass_kg,
            )
            _, metrics, _ = run_simulation(
                duration_s=duration_s,
                dt_s=0.05,
                guidance_mode=guidance_mode,
                navigation_mode="ekf",
                actuator_mode="flight_like",
                initial_state=initial,
                rng_seed=seed,
            )
            rows.append(
                {
                    "initial_x_m": initial_x_m,
                    "guidance_mode": guidance_mode,
                    "success": metrics["success"],
                    "landing_x_error_m": metrics["landing_x_error_m"],
                    "touchdown_speed_mps": metrics["touchdown_speed_mps"],
                    "propellant_remaining_kg": metrics[
                        "propellant_remaining_kg"
                    ],
                    "max_abs_tilt_deg": metrics["max_abs_tilt_deg"],
                }
            )
    return rows


def write_dict_csv(rows: list[dict], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=list(rows[0].keys()),
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
