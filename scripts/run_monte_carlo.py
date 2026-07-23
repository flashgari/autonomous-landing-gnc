#!/usr/bin/env python3
"""Run the landing Monte Carlo dispersion campaign."""

import argparse
from pathlib import Path

from landing_gnc.monte_carlo import compare_summaries, run_monte_carlo, write_monte_carlo_outputs
from landing_gnc.sim import write_json


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=200)
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--dt", type=float, default=0.08)
    parser.add_argument("--mode", choices=("baseline", "corridor", "both"), default="baseline")
    parser.add_argument("--duration", type=float, default=70.0)
    args = parser.parse_args()

    modes = ["baseline", "corridor"] if args.mode == "both" else [args.mode]
    summaries = []
    for mode in modes:
        rows, summary = run_monte_carlo(
            num_cases=args.cases,
            seed=args.seed,
            dt_s=args.dt,
            guidance_mode=mode,
            duration_s=args.duration,
        )
        write_monte_carlo_outputs(rows, summary)
        suffix = "" if mode == "baseline" else f"_{mode}"
        print(f"Wrote outputs/monte_carlo_landing{suffix}.csv")
        print(f"Wrote outputs/monte_carlo_summary{suffix}.json")
        for key, value in summary.items():
            print(f"{mode}.{key}: {value}")
        summaries.append(summary)

    if len(summaries) > 1:
        comparison = compare_summaries(summaries)
        write_json(comparison, Path("outputs/monte_carlo_guidance_comparison.json"))
        print("Wrote outputs/monte_carlo_guidance_comparison.json")


if __name__ == "__main__":
    main()
