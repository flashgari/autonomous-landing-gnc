#!/usr/bin/env python3
"""Plot actuator, fault, and hazard-divert scenario results."""

import csv
import json
from pathlib import Path


def load_history(name):
    with Path(f"outputs/scenario_{name}.csv").open() as f:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(f)]


def main():
    summary = json.loads(Path("outputs/advanced_scenarios.json").read_text())
    histories = {name: load_history(name) for name in summary["scenarios"]}
    write_svg(summary, histories, Path("figures/advanced_scenario_comparison.svg"))
    print("Wrote figures/advanced_scenario_comparison.svg")


def write_svg(summary, histories, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 1040, 720
    plot_x, plot_y, plot_w, plot_h = 82, 108, 580, 390
    names = list(summary["scenarios"])
    labels = {
        "full_stack_nominal": "full-stack nominal",
        "thrust_loss": "18% thrust loss",
        "thrust_loss_recoverable": "8% thrust loss",
        "altitude_sensor_fault": "altitude bias fault",
        "hazard_divert": "hazard divert",
    }
    colors = {
        "full_stack_nominal": "#2563eb",
        "thrust_loss": "#dc2626",
        "thrust_loss_recoverable": "#ea580c",
        "altitude_sensor_fault": "#7c3aed",
        "hazard_divert": "#059669",
    }
    all_x = [row["x_m"] for rows in histories.values() for row in rows]
    all_z = [row["z_m"] for rows in histories.values() for row in rows]
    x_min, x_max = min(all_x) - 3.0, max(all_x) + 3.0
    z_max = max(all_z)

    def sx(x):
        return plot_x + (x - x_min) / (x_max - x_min) * plot_w

    def sy(z):
        return plot_y + plot_h - z / z_max * plot_h

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Fault response and hazard divert scenario comparison</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="52" y="42" font-family="Arial" font-size="24" font-weight="700" fill="#0f172a">Actuator, Fault, and Hazard-Divert Verification</text>',
        '<text x="52" y="67" font-family="Arial" font-size="14" fill="#475569">Corridor guidance uses estimated state and flight-like command-path dynamics in every case.</text>',
        f'<rect x="{plot_x}" y="{plot_y}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#cbd5e1"/>',
    ]
    hazard = summary["hazards"][0]
    svg.append(f'<rect x="{sx(hazard["left_m"]):.1f}" y="{sy(28):.1f}" width="{sx(hazard["right_m"])-sx(hazard["left_m"]):.1f}" height="{sy(0)-sy(28):.1f}" fill="#fee2e2" stroke="#dc2626" opacity="0.80"/>')
    svg.append(f'<text x="{sx(hazard["left_m"])+4:.1f}" y="{sy(28)-7:.1f}" font-family="Arial" font-size="12" fill="#991b1b">debris hazard</text>')
    legend_y = plot_y + 20
    for name in names:
        rows = histories[name]
        path_data = " ".join(
            ("M" if i == 0 else "L") + f"{sx(row['x_m']):.1f},{sy(row['z_m']):.1f}"
            for i, row in enumerate(rows)
        )
        svg.append(f'<path d="{path_data}" fill="none" stroke="{colors[name]}" stroke-width="2.5"/>')
        svg.append(f'<line x1="{plot_x+14}" y1="{legend_y}" x2="{plot_x+38}" y2="{legend_y}" stroke="{colors[name]}" stroke-width="3"/>')
        svg.append(f'<text x="{plot_x+45}" y="{legend_y+4}" font-family="Arial" font-size="12" fill="#334155">{labels[name]}</text>')
        legend_y += 23
    svg.append(f'<text x="{plot_x+plot_w/2-42}" y="{plot_y+plot_h+38}" font-family="Arial" font-size="14" fill="#334155">downrange x (m)</text>')
    svg.append(f'<text x="22" y="{plot_y+plot_h/2+24}" font-family="Arial" font-size="14" fill="#334155" transform="rotate(-90 22,{plot_y+plot_h/2+24})">altitude z (m)</text>')

    side_x = 710
    svg.append(f'<text x="{side_x}" y="110" font-family="Arial" font-size="18" font-weight="700" fill="#0f172a">Terminal constraints</text>')
    y = 142
    for name in names:
        metrics = summary["scenarios"][name]["metrics"]
        status = "PASS" if metrics["success"] else "FAIL"
        svg.append(f'<text x="{side_x}" y="{y}" font-family="Arial" font-size="14" font-weight="700" fill="#0f172a">{labels[name]}</text>')
        svg.append(f'<text x="{side_x+232}" y="{y}" font-family="Arial" font-size="13" font-weight="700" fill="{colors[name]}">{status}</text>')
        svg.append(f'<text x="{side_x}" y="{y+20}" font-family="Arial" font-size="12" fill="#475569">target error {metrics["landing_x_error_m"]:.2f} m | speed {metrics["touchdown_speed_mps"]:.2f} m/s</text>')
        svg.append(f'<text x="{side_x}" y="{y+38}" font-family="Arial" font-size="12" fill="#475569">propellant {metrics["propellant_remaining_kg"]:.0f} kg | nav rejects {metrics["navigation_rejection_count"]}</text>')
        y += 66

    nominal_prop = summary["scenarios"]["full_stack_nominal"]["metrics"]["propellant_remaining_kg"]
    divert = summary["scenarios"]["hazard_divert"]["metrics"]
    thrust = summary["scenarios"]["thrust_loss"]["metrics"]
    svg.append(f'<text x="52" y="{height-72}" font-family="Arial" font-size="14" font-weight="700" fill="#0f172a">Physical result</text>')
    svg.append(f'<text x="52" y="{height-48}" font-family="Arial" font-size="13" fill="#475569">The hazard divert lands {divert["hazard_clearance_m"]:.2f} m clear of the debris zone and changes remaining propellant by {divert["propellant_remaining_kg"]-nominal_prop:+.0f} kg.</text>')
    svg.append(f'<text x="52" y="{height-27}" font-family="Arial" font-size="13" fill="#475569">The 18% thrust-loss case crosses the terminal footprint boundary: error {thrust["landing_x_error_m"]:.2f} m, exposing loss of divert authority before propellant depletion.</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
