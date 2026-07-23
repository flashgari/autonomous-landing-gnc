#!/usr/bin/env python3
"""Map landing success and propellant margin over altitude/divert conditions."""

from pathlib import Path

from landing_gnc.experiments import run_feasibility_grid, summarize_feasibility
from landing_gnc.monte_carlo import write_monte_carlo_csv
from landing_gnc.sim import write_json


def main():
    rows = run_feasibility_grid()
    summary = summarize_feasibility(rows)
    write_monte_carlo_csv(rows, Path("outputs/landing_feasibility_envelope.csv"))
    write_json(summary, Path("outputs/landing_feasibility_summary.json"))
    print("Wrote landing feasibility envelope outputs")
    print(summary)


if __name__ == "__main__":
    main()
