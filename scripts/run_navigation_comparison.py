#!/usr/bin/env python3
"""Compare truth-state and estimated-state feedback on identical dispersions."""

import argparse
from pathlib import Path

from landing_gnc.monte_carlo import run_monte_carlo, write_monte_carlo_csv
from landing_gnc.sim import run_simulation, write_csv, write_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=200)
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--dt", type=float, default=0.08)
    parser.add_argument("--duration", type=float, default=70.0)
    args = parser.parse_args()

    nominal_rows, nominal_metrics, nominal_config = run_simulation(
        duration_s=args.duration,
        dt_s=0.05,
        guidance_mode="corridor",
        navigation_mode="estimated",
        actuator_mode="flight_like",
        rng_seed=args.seed,
    )
    write_csv(nominal_rows, Path("outputs/nominal_landing_estimated.csv"))
    write_json(nominal_metrics, Path("outputs/nominal_landing_estimated_metrics.json"))
    write_json(nominal_config, Path("outputs/nominal_landing_estimated_config.json"))

    summaries = {}
    for navigation_mode in ("truth", "estimated"):
        rows, summary = run_monte_carlo(
            num_cases=args.cases,
            seed=args.seed,
            dt_s=args.dt,
            duration_s=args.duration,
            guidance_mode="corridor",
            navigation_mode=navigation_mode,
            actuator_mode="flight_like",
        )
        suffix = f"corridor_{navigation_mode}_actuated"
        write_monte_carlo_csv(rows, Path(f"outputs/monte_carlo_landing_{suffix}.csv"))
        write_json(summary, Path(f"outputs/monte_carlo_summary_{suffix}.json"))
        summaries[navigation_mode] = summary

    truth = summaries["truth"]
    estimated = summaries["estimated"]
    comparison = {
        "summaries": summaries,
        "delta_estimated_minus_truth": {
            "success_rate_points": 100.0 * (estimated["success_rate"] - truth["success_rate"]),
            "p95_abs_landing_error_m": estimated["p95_abs_landing_error_m"]
            - truth["p95_abs_landing_error_m"],
            "p95_touchdown_speed_mps": estimated["p95_touchdown_speed_mps"]
            - truth["p95_touchdown_speed_mps"],
        },
        "nominal_estimation_metrics": nominal_metrics,
    }
    write_json(comparison, Path("outputs/navigation_comparison.json"))
    print("Wrote estimated-state nominal and Monte Carlo comparison outputs")
    print(comparison)


if __name__ == "__main__":
    main()
