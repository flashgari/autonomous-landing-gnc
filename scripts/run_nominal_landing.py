#!/usr/bin/env python3
"""Run the nominal autonomous landing simulation."""

from pathlib import Path

from landing_gnc.sim import run_simulation, write_csv, write_json


def main():
    rows, metrics, config = run_simulation()
    write_csv(rows, Path("outputs/nominal_landing.csv"))
    write_json(metrics, Path("outputs/nominal_landing_metrics.json"))
    write_json(config, Path("outputs/nominal_landing_config.json"))
    print("Wrote outputs/nominal_landing.csv")
    print("Wrote outputs/nominal_landing_metrics.json")
    for key, value in metrics.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()

