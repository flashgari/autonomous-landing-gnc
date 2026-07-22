#!/usr/bin/env python3
"""Run the landing Monte Carlo dispersion campaign."""

import argparse

from landing_gnc.monte_carlo import run_monte_carlo, write_monte_carlo_outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cases", type=int, default=200)
    parser.add_argument("--seed", type=int, default=4242)
    parser.add_argument("--dt", type=float, default=0.08)
    args = parser.parse_args()

    rows, summary = run_monte_carlo(num_cases=args.cases, seed=args.seed, dt_s=args.dt)
    write_monte_carlo_outputs(rows, summary)
    print("Wrote outputs/monte_carlo_landing.csv")
    print("Wrote outputs/monte_carlo_summary.json")
    for key, value in summary.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()

