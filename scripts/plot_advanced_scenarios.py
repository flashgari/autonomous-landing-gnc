#!/usr/bin/env python3
"""Plot actuator, fault, and hazard-divert scenario results."""

import csv
import json
import math
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
    width, height = 1200, 900
    plot_x, plot_y, plot_w, plot_h = 72, 132, 680, 470
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
    x_min = 5.0 * math.floor((min(all_x) - 3.0) / 5.0)
    x_max = 5.0 * math.ceil((max(all_x) + 3.0) / 5.0)
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
        '<text x="52" y="67" font-family="Arial" font-size="14" fill="#475569">Identical initial state, estimator, corridor guidance, and finite-rate command path; only the stated fault or target changes.</text>',
        f'<rect x="{plot_x}" y="{plot_y}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#cbd5e1"/>',
    ]
    z_ticks = [0.0, 180.0, 360.0, 540.0, 720.0]
    for tick in z_ticks:
        y_tick = sy(tick)
        svg.append(f'<line x1="{plot_x}" y1="{y_tick:.1f}" x2="{plot_x+plot_w}" y2="{y_tick:.1f}" stroke="#e2e8f0"/>')
        svg.append(f'<text x="{plot_x-10}" y="{y_tick+4:.1f}" text-anchor="end" font-family="Arial" font-size="11" fill="#64748b">{tick:.0f}</text>')
    x_tick = x_min
    while x_tick <= x_max + 1e-9:
        x_tick_px = sx(x_tick)
        svg.append(f'<line x1="{x_tick_px:.1f}" y1="{plot_y}" x2="{x_tick_px:.1f}" y2="{plot_y+plot_h}" stroke="#f1f5f9"/>')
        svg.append(f'<text x="{x_tick_px:.1f}" y="{plot_y+plot_h+17}" text-anchor="middle" font-family="Arial" font-size="11" fill="#64748b">{x_tick:.0f}</text>')
        x_tick += 5.0
    hazard = summary["hazards"][0]
    svg.append(f'<rect x="{sx(hazard["left_m"]):.1f}" y="{sy(26):.1f}" width="{sx(hazard["right_m"])-sx(hazard["left_m"]):.1f}" height="{sy(0)-sy(26):.1f}" fill="#fee2e2" stroke="#dc2626" opacity="0.86"/>')
    svg.append(f'<rect x="{sx(0.0)-9:.1f}" y="{sy(0)-5:.1f}" width="18" height="5" fill="#2563eb"/>')
    svg.append(f'<text x="{sx(0.0):.1f}" y="{plot_y+plot_h+34}" text-anchor="middle" font-family="Arial" font-size="10.5" fill="#2563eb">nominal target</text>')
    divert_target = summary["scenarios"]["hazard_divert"]["metrics"]["target_x_m"]
    svg.append(f'<rect x="{sx(divert_target)-9:.1f}" y="{sy(0)-5:.1f}" width="18" height="5" fill="#059669"/>')
    svg.append(f'<text x="{sx(divert_target):.1f}" y="{plot_y+plot_h+34}" text-anchor="middle" font-family="Arial" font-size="10.5" fill="#059669">divert target</text>')
    svg.append(f'<rect x="{plot_x+12}" y="{plot_y+10}" width="188" height="148" rx="3" fill="#ffffff" opacity="0.94" stroke="#e2e8f0"/>')
    legend_y = plot_y + 29
    for name in names:
        rows = histories[name]
        path_data = " ".join(
            ("M" if i == 0 else "L") + f"{sx(row['x_m']):.1f},{sy(row['z_m']):.1f}"
            for i, row in enumerate(rows)
        )
        svg.append(f'<path d="{path_data}" fill="none" stroke="{colors[name]}" stroke-width="2.5"/>')
        final = rows[-1]
        svg.append(f'<circle cx="{sx(final["x_m"]):.1f}" cy="{sy(final["z_m"]):.1f}" r="4" fill="{colors[name]}" stroke="#ffffff" stroke-width="1"/>')
        svg.append(f'<line x1="{plot_x+24}" y1="{legend_y}" x2="{plot_x+48}" y2="{legend_y}" stroke="{colors[name]}" stroke-width="3"/>')
        svg.append(f'<text x="{plot_x+55}" y="{legend_y+4}" font-family="Arial" font-size="11.5" fill="#334155">{labels[name]}</text>')
        legend_y += 22
    svg.append(f'<rect x="{plot_x+24}" y="{legend_y-7}" width="24" height="10" fill="#fee2e2" stroke="#dc2626"/>')
    svg.append(f'<text x="{plot_x+55}" y="{legend_y+3}" font-family="Arial" font-size="11.5" fill="#334155">debris interval [-4, 4] m</text>')
    svg.append(f'<text x="{plot_x+plot_w/2}" y="{plot_y+plot_h+57}" text-anchor="middle" font-family="Arial" font-size="14" fill="#334155">downrange x (m)</text>')
    svg.append(f'<text x="22" y="{plot_y+plot_h/2+24}" font-family="Arial" font-size="14" fill="#334155" transform="rotate(-90 22,{plot_y+plot_h/2+24})">altitude z (m)</text>')

    side_x = 790
    svg.append(f'<text x="{side_x}" y="126" font-family="Arial" font-size="18" font-weight="700" fill="#0f172a">Terminal constraint audit</text>')
    svg.append(f'<rect x="{side_x}" y="140" width="358" height="48" rx="4" fill="#ffffff" stroke="#cbd5e1"/>')
    svg.append(f'<text x="{side_x+12}" y="160" font-family="Arial" font-size="11" font-weight="700" fill="#0f172a">PASS requires every condition</text>')
    svg.append(f'<text x="{side_x+12}" y="178" font-family="Arial" font-size="10.5" fill="#475569">|e_x| &lt; 3 m; |v_x| &lt; 1 m/s; |v_z| &lt; 2.5 m/s; |theta|max &lt; 12 deg; propellant &gt; 0</text>')
    y = 214
    for name in names:
        metrics = summary["scenarios"][name]["metrics"]
        status = "PASS" if metrics["success"] else "FAIL"
        svg.append(f'<text x="{side_x}" y="{y}" font-family="Arial" font-size="14" font-weight="700" fill="#0f172a">{labels[name]}</text>')
        svg.append(f'<text x="{side_x+316}" y="{y}" font-family="Arial" font-size="13" font-weight="700" fill="{colors[name]}">{status}</text>')
        svg.append(f'<text x="{side_x}" y="{y+18}" font-family="Arial" font-size="11.5" fill="#475569">e_x {metrics["landing_x_error_m"]:.2f} m | v_x {metrics["touchdown_vx_mps"]:.2f} m/s | v_z {metrics["touchdown_vz_mps"]:.2f} m/s</text>')
        svg.append(f'<text x="{side_x}" y="{y+35}" font-family="Arial" font-size="11.5" fill="#475569">prop {metrics["propellant_remaining_kg"]:.0f} kg | t_f {metrics["final_time_s"]:.1f} s | nav rejects {metrics["navigation_rejection_count"]}</text>')
        y += 66

    nominal_prop = summary["scenarios"]["full_stack_nominal"]["metrics"]["propellant_remaining_kg"]
    nominal_time = summary["scenarios"]["full_stack_nominal"]["metrics"]["final_time_s"]
    divert = summary["scenarios"]["hazard_divert"]["metrics"]
    thrust = summary["scenarios"]["thrust_loss"]["metrics"]
    sensor = summary["scenarios"]["altitude_sensor_fault"]["metrics"]
    bottom_y = 690
    box_w = 350
    boxes = [
        (
            52,
            "Trajectory geometry",
            [
                "The green S-curve is a lateral impulse-and-brake sequence:",
                "early tilt accelerates toward the safe site; opposite tilt later",
                f"removes crossrange velocity. Touchdown is {divert['hazard_clearance_m']:.2f} m clear.",
            ],
        ),
        (
            425,
            "Thrust-authority boundary",
            [
                "An 8% loss remains inside the available impulse/time-to-go",
                f"margin. At 18% loss, flight ends {nominal_time-thrust['final_time_s']:.1f} s earlier and e_x",
                f"is {thrust['landing_x_error_m']:.2f} m despite {thrust['propellant_remaining_kg']:.0f} kg remaining.",
            ],
        ),
        (
            798,
            "Navigation-fault response",
            [
                f"The +12 m altitude step produces {sensor['navigation_rejection_count']} gated innovations.",
                f"Estimator consistency is retained, but flight extends to {sensor['final_time_s']:.1f} s,",
                f"adding gravity loss and using {nominal_prop-sensor['propellant_remaining_kg']:.0f} kg vs nominal.",
            ],
        ),
    ]
    for x, heading, lines in boxes:
        svg.append(f'<rect x="{x}" y="{bottom_y}" width="{box_w}" height="150" rx="4" fill="#ffffff" stroke="#cbd5e1"/>')
        svg.append(f'<text x="{x+16}" y="{bottom_y+25}" font-family="Arial" font-size="14" font-weight="700" fill="#0f172a">{heading}</text>')
        for line_index, line in enumerate(lines):
            svg.append(f'<text x="{x+16}" y="{bottom_y+52+21*line_index}" font-family="Arial" font-size="11.5" fill="#334155">{line}</text>')
    svg.append(f'<text x="52" y="872" font-family="Arial" font-size="11.5" fill="#475569">Interpretation: positive propellant is an inventory margin, not a guarantee of finite-time reachable acceleration after thrust loss or late retargeting.</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
