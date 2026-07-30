#!/usr/bin/env python3
"""Run deterministic 3D 6-DOF landing verification cases."""

from pathlib import Path

from landing_gnc.sixdof_models import (
    SixDofEnvironment,
    SixDofScenario,
)
from landing_gnc.sixdof_sim import (
    run_sixdof_simulation,
    write_csv,
    write_json,
)


CASES = {
    "nominal": (
        SixDofEnvironment(wind_inertial_mps=(0.0, 0.0, 0.0)),
        SixDofScenario(name="nominal"),
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


def main() -> None:
    summary = {
        "model": "14-state 3D rigid-body powered-descent plant",
        "state_definition": (
            "r_I[3], v_I[3], q_BI[4], omega_B[3], mass[1]"
        ),
        "frame_convention": {
            "inertial": "x/y horizontal, z positive upward",
            "body": "+z along the nominal thrust axis",
            "quaternion": "scalar-first rotation from body to inertial",
        },
        "acceptance_criteria": {
            "horizontal_target_error_m": 3.0,
            "horizontal_touchdown_speed_mps": 1.0,
            "vertical_touchdown_speed_mps": 2.5,
            "maximum_tilt_deg": 12.0,
            "positive_propellant_required": True,
        },
        "cases": {},
    }
    for name, (environment, scenario) in CASES.items():
        rows, metrics, configuration = run_sixdof_simulation(
            duration_s=55.0,
            dt_s=0.05,
            environment=environment,
            scenario=scenario,
        )
        write_csv(
            rows,
            Path(f"outputs/sixdof_{name}.csv"),
        )
        write_json(
            configuration,
            Path(f"outputs/sixdof_{name}_config.json"),
        )
        summary["cases"][name] = {
            "metrics": metrics,
            "wind_inertial_mps": list(
                environment.wind_inertial_mps
            ),
            "failed_engine_index": scenario.failed_engine_index,
            "engine_failure_time_s": (
                scenario.engine_failure_time_s
                if scenario.failed_engine_index is not None
                else None
            ),
        }
        print(
            f"{name}: success={metrics['success']}, "
            f"error={metrics['horizontal_target_error_m']:.2f} m, "
            f"vertical speed="
            f"{metrics['touchdown_vertical_speed_mps']:.2f} m/s, "
            f"max tilt={metrics['maximum_tilt_deg']:.2f} deg, "
            f"propellant={metrics['propellant_remaining_kg']:.0f} kg"
        )
    write_json(
        summary,
        Path("outputs/sixdof_verification_summary.json"),
    )
    print("Wrote 3D 6-DOF trajectories and verification summary")


if __name__ == "__main__":
    main()
