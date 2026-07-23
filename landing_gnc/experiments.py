"""Repeatable higher-level verification experiments."""

import math
from dataclasses import replace

from .hazards import point_clearance_m, select_safe_target
from .models import FaultScenario, HazardZone, State, Vehicle
from .sim import default_initial_state, run_simulation


def run_fault_and_divert_scenarios(dt_s: float = 0.05, duration_s: float = 70.0) -> dict:
    vehicle = Vehicle()
    initial = default_initial_state(vehicle)
    scenarios = {
        "full_stack_nominal": {
            "navigation_mode": "estimated",
            "actuator_mode": "flight_like",
            "fault": FaultScenario(name="none"),
            "target_x_m": 0.0,
            "rng_seed": 19,
        },
        "thrust_loss": {
            "navigation_mode": "estimated",
            "actuator_mode": "flight_like",
            "fault": FaultScenario(
                name="cluster_engine_out_equivalent",
                thrust_loss_time_s=5.0,
                thrust_scale_after_fault=0.82,
            ),
            "target_x_m": 0.0,
            "rng_seed": 19,
        },
        "thrust_loss_recoverable": {
            "navigation_mode": "estimated",
            "actuator_mode": "flight_like",
            "fault": FaultScenario(
                name="recoverable_thrust_decrement",
                thrust_loss_time_s=5.0,
                thrust_scale_after_fault=0.92,
            ),
            "target_x_m": 0.0,
            "rng_seed": 19,
        },
        "altitude_sensor_fault": {
            "navigation_mode": "estimated",
            "actuator_mode": "flight_like",
            "fault": FaultScenario(
                name="altitude_bias_step",
                altitude_bias_step_time_s=7.0,
                altitude_bias_step_m=12.0,
            ),
            "target_x_m": 0.0,
            "rng_seed": 19,
        },
    }

    hazards = (HazardZone(-4.0, 4.0, "debris field"),)
    divert_target = select_safe_target(initial.x_m, (-16.0, 0.0, 12.0), hazards)
    scenarios["hazard_divert"] = {
        "navigation_mode": "estimated",
        "actuator_mode": "flight_like",
        "fault": FaultScenario(name="none"),
        "target_x_m": divert_target,
        "rng_seed": 19,
    }

    results = {}
    histories = {}
    for name, settings in scenarios.items():
        rows, metrics, config = run_simulation(
            duration_s=duration_s,
            dt_s=dt_s,
            vehicle=vehicle,
            initial_state=replace(initial),
            guidance_mode="corridor",
            **settings,
        )
        if name == "hazard_divert":
            metrics["hazard_clearance_m"] = point_clearance_m(metrics["landing_x_m"], hazards)
            metrics["divert_distance_m"] = abs(divert_target)
        results[name] = {"metrics": metrics, "config": config}
        histories[name] = rows
    return {"scenarios": results, "hazards": [zone.__dict__ for zone in hazards]}, histories


def run_feasibility_grid(
    altitudes_m=(300.0, 450.0, 600.0, 750.0, 900.0),
    offsets_m=(0.0, 10.0, 20.0, 30.0, 40.0, 50.0),
    dt_s: float = 0.08,
    duration_s: float = 80.0,
) -> list[dict]:
    vehicle = Vehicle()
    rows = []
    for altitude_m in altitudes_m:
        for offset_m in offsets_m:
            initial_descent_speed = -min(
                58.0,
                0.94 * math.sqrt(2.0 * 2.2 * altitude_m),
            )
            initial = State(
                time_s=0.0,
                x_m=offset_m,
                z_m=altitude_m,
                vx_mps=-1.5,
                vz_mps=initial_descent_speed,
                theta_rad=math.radians(0.8),
                omega_radps=0.0,
                mass_kg=vehicle.wet_mass_kg,
            )
            _, metrics, _ = run_simulation(
                duration_s=duration_s,
                dt_s=dt_s,
                vehicle=vehicle,
                initial_state=initial,
                guidance_mode="corridor",
                navigation_mode="truth",
                actuator_mode="flight_like",
            )
            rows.append(
                {
                    "initial_altitude_m": altitude_m,
                    "initial_offset_m": offset_m,
                    "initial_vz_mps": initial_descent_speed,
                    **metrics,
                }
            )
    return rows


def summarize_feasibility(rows: list[dict]) -> dict:
    successes = [row for row in rows if row["success"]]
    return {
        "num_cases": len(rows),
        "successes": len(successes),
        "success_rate": len(successes) / len(rows),
        "maximum_successful_offset_m": max(
            (row["initial_offset_m"] for row in successes),
            default=0.0,
        ),
        "minimum_successful_altitude_m": min(
            (row["initial_altitude_m"] for row in successes),
            default=0.0,
        ),
        "minimum_propellant_remaining_kg": min(
            (row["propellant_remaining_kg"] for row in successes),
            default=0.0,
        ),
    }


def run_divert_cost_sweep(
    targets_m=(-12.0, 0.0, 12.0, 24.0),
    dt_s: float = 0.05,
    duration_s: float = 70.0,
) -> list[dict]:
    vehicle = Vehicle()
    initial = default_initial_state(vehicle)
    rows = []
    for target_x_m in targets_m:
        _, metrics, _ = run_simulation(
            duration_s=duration_s,
            dt_s=dt_s,
            vehicle=vehicle,
            initial_state=replace(initial),
            guidance_mode="corridor",
            navigation_mode="estimated",
            actuator_mode="flight_like",
            rng_seed=19,
            target_x_m=target_x_m,
        )
        rows.append(
            {
                "target_x_m": target_x_m,
                "required_lateral_correction_m": abs(initial.x_m - target_x_m),
                **metrics,
            }
        )
    return rows
