#!/usr/bin/env python3
"""Run deterministic ESKF fault cases and matched-seed Monte Carlo comparison."""

import argparse
from pathlib import Path

from landing_gnc.models import FaultScenario
from landing_gnc.monte_carlo import run_monte_carlo, write_monte_carlo_csv
from landing_gnc.sim import run_simulation, write_csv, write_json


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=200)
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--dt", type=float, default=0.08)
    parser.add_argument("--duration", type=float, default=70.0)
    args = parser.parse_args()

    deterministic_faults = {
        "nominal": FaultScenario(name="nominal"),
        "gps_dropout": FaultScenario(
            name="gps_dropout",
            gps_dropout_start_s=8.0,
            gps_dropout_end_s=28.0,
        ),
        "radar_bias": FaultScenario(
            name="radar_bias",
            altitude_bias_step_time_s=12.0,
            altitude_bias_step_m=12.0,
        ),
    }
    deterministic_metrics = {}
    for case_name, fault in deterministic_faults.items():
        rows, metrics, config = run_simulation(
            duration_s=args.duration,
            dt_s=0.05,
            guidance_mode="corridor",
            navigation_mode="ekf",
            actuator_mode="flight_like",
            rng_seed=args.seed,
            fault=fault,
        )
        write_csv(rows, Path(f"outputs/ekf_{case_name}.csv"))
        write_json(config, Path(f"outputs/ekf_{case_name}_config.json"))
        deterministic_metrics[case_name] = metrics

    summaries = {}
    for navigation_mode in ("estimated", "ekf"):
        rows, summary = run_monte_carlo(
            num_cases=args.cases,
            seed=args.seed,
            dt_s=args.dt,
            duration_s=args.duration,
            guidance_mode="corridor",
            navigation_mode=navigation_mode,
            actuator_mode="flight_like",
        )
        write_monte_carlo_csv(
            rows,
            Path(f"outputs/monte_carlo_navigation_{navigation_mode}.csv"),
        )
        write_json(
            summary,
            Path(f"outputs/monte_carlo_navigation_{navigation_mode}_summary.json"),
        )
        summaries[navigation_mode] = summary

    alpha_beta = summaries["estimated"]
    ekf = summaries["ekf"]
    comparison = {
        "campaign": {
            "cases_per_filter": args.cases,
            "seed": args.seed,
            "simulation_dt_s": args.dt,
            "deterministic_dt_s": 0.05,
        },
        "deterministic_cases": deterministic_metrics,
        "monte_carlo_summaries": summaries,
        "delta_ekf_minus_alpha_beta": {
            "success_rate_points": 100.0
            * (ekf["success_rate"] - alpha_beta["success_rate"]),
            "p95_abs_landing_error_m": ekf["p95_abs_landing_error_m"]
            - alpha_beta["p95_abs_landing_error_m"],
            "p95_touchdown_speed_mps": ekf["p95_touchdown_speed_mps"]
            - alpha_beta["p95_touchdown_speed_mps"],
        },
    }
    write_json(comparison, Path("outputs/ekf_navigation_campaign.json"))
    print("Wrote ESKF deterministic and Monte Carlo campaign outputs")
    print(comparison)


if __name__ == "__main__":
    main()
