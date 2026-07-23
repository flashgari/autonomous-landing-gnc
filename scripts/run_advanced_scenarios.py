#!/usr/bin/env python3
"""Run deterministic actuator, fault, and hazard-divert verification cases."""

from pathlib import Path

from landing_gnc.experiments import run_divert_cost_sweep, run_fault_and_divert_scenarios
from landing_gnc.monte_carlo import write_monte_carlo_csv
from landing_gnc.sim import write_csv, write_json


def main():
    summary, histories = run_fault_and_divert_scenarios()
    divert_sweep = run_divert_cost_sweep()
    summary["divert_sweep"] = divert_sweep
    write_json(summary, Path("outputs/advanced_scenarios.json"))
    write_monte_carlo_csv(divert_sweep, Path("outputs/divert_cost_sweep.csv"))
    for name, rows in histories.items():
        write_csv(rows, Path(f"outputs/scenario_{name}.csv"))
    print("Wrote outputs/advanced_scenarios.json and scenario histories")
    for name, result in summary["scenarios"].items():
        metrics = result["metrics"]
        print(
            f"{name}: success={metrics['success']}, "
            f"error={metrics['landing_x_error_m']:.2f} m, "
            f"speed={metrics['touchdown_speed_mps']:.2f} m/s, "
            f"prop={metrics['propellant_remaining_kg']:.0f} kg"
        )


if __name__ == "__main__":
    main()
