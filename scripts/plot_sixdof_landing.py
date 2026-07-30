#!/usr/bin/env python3
"""Generate recruiter-facing 3D 6-DOF landing verification evidence."""

import csv
import html
import json
import math
from pathlib import Path


COLORS = {
    "nominal": "#2563eb",
    "crosswind": "#059669",
    "high_crosswind": "#ea580c",
    "engine_out": "#dc2626",
}
LABELS = {
    "nominal": "calm nominal",
    "crosswind": "12/-6 m/s crosswind",
    "high_crosswind": "18/-10 m/s crosswind",
    "engine_out": "engine 1 out at 12 s",
}


def load_rows(name):
    with Path(f"outputs/sixdof_{name}.csv").open() as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def line_path(rows, x_value, y_value, sx, sy):
    return " ".join(
        (
            "M" if index == 0 else "L"
        )
        + f"{sx(x_value(row)):.1f},{sy(y_value(row)):.1f}"
        for index, row in enumerate(rows)
    )


def text(svg, x, y, value, size=12, color="#334155", weight=400, anchor="start"):
    escaped_value = html.escape(str(value))
    svg.append(
        f'<text x="{x}" y="{y}" text-anchor="{anchor}" '
        f'font-family="Arial,Helvetica,sans-serif" font-size="{size}" '
        f'font-weight="{weight}" fill="{color}">{escaped_value}</text>'
    )


def panel(svg, x, y, width, height, title, subtitle=None):
    svg.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        'rx="4" fill="#ffffff" stroke="#cbd5e1"/>'
    )
    text(svg, x + 18, y + 29, title, 16, "#0f172a", 700)
    if subtitle:
        text(svg, x + 18, y + 49, subtitle, 11, "#64748b")


def main():
    summary = json.loads(
        Path("outputs/sixdof_verification_summary.json").read_text()
    )
    names = [
        "nominal",
        "crosswind",
        "high_crosswind",
        "engine_out",
    ]
    histories = {name: load_rows(name) for name in names}
    write_svg(
        summary,
        histories,
        Path("figures/sixdof_landing_verification.svg"),
    )
    print("Wrote figures/sixdof_landing_verification.svg")


def write_svg(summary, histories, path):
    width, height = 1440, 1180
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>3D six degree of freedom landing verification</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
    ]
    text(
        svg,
        52,
        48,
        "3D 6-DOF Powered-Descent Verification",
        28,
        "#0f172a",
        700,
    )
    text(
        svg,
        52,
        76,
        (
            "14-state nonlinear rigid body, quaternion attitude, "
            "mass-scheduled inertia, four-engine allocation, and finite-rate actuators"
        ),
        14,
        "#475569",
    )
    legend_x = 52
    for name in histories:
        svg.append(
            f'<line x1="{legend_x}" y1="112" x2="{legend_x+30}" '
            f'y2="112" stroke="{COLORS[name]}" stroke-width="4"/>'
        )
        text(svg, legend_x + 39, 117, LABELS[name], 12, "#334155", 600)
        legend_x += 275 if name != "nominal" else 190

    ground_x, ground_y, ground_w, ground_h = 52, 148, 610, 452
    panel(
        svg,
        ground_x,
        ground_y,
        ground_w,
        ground_h,
        "A. Horizontal ground track",
        "Independent inertial x/y translation; dot marks touchdown",
    )
    gx, gy = ground_x + 62, ground_y + 72
    gw, gh = ground_w - 92, ground_h - 112
    all_x = [row["x_m"] for rows in histories.values() for row in rows]
    all_y = [row["y_m"] for rows in histories.values() for row in rows]
    low = 5.0 * math.floor((min(all_x + all_y) - 2.0) / 5.0)
    high = 5.0 * math.ceil((max(all_x + all_y) + 2.0) / 5.0)
    span = max(5.0, high - low)
    sx_ground = lambda value: gx + (value - low) / span * gw
    sy_ground = lambda value: gy + gh - (value - low) / span * gh
    tick = low
    while tick <= high + 1.0e-9:
        px, py = sx_ground(tick), sy_ground(tick)
        svg.append(
            f'<line x1="{px:.1f}" y1="{gy}" x2="{px:.1f}" '
            f'y2="{gy+gh}" stroke="#e2e8f0"/>'
        )
        svg.append(
            f'<line x1="{gx}" y1="{py:.1f}" x2="{gx+gw}" '
            f'y2="{py:.1f}" stroke="#e2e8f0"/>'
        )
        text(svg, px, gy + gh + 20, f"{tick:.0f}", 10, "#64748b", anchor="middle")
        text(svg, gx - 10, py + 4, f"{tick:.0f}", 10, "#64748b", anchor="end")
        tick += 5.0
    svg.append(
        f'<circle cx="{sx_ground(0):.1f}" cy="{sy_ground(0):.1f}" '
        'r="11" fill="none" stroke="#0f172a" stroke-width="2"/>'
    )
    svg.append(
        f'<circle cx="{sx_ground(0):.1f}" cy="{sy_ground(0):.1f}" '
        'r="3" fill="#0f172a"/>'
    )
    text(
        svg,
        sx_ground(0) + 15,
        sy_ground(0) - 9,
        "target",
        10,
        "#0f172a",
        700,
    )
    for name, rows in histories.items():
        path_data = line_path(
            rows,
            lambda row: row["x_m"],
            lambda row: row["y_m"],
            sx_ground,
            sy_ground,
        )
        svg.append(
            f'<path d="{path_data}" fill="none" stroke="{COLORS[name]}" '
            'stroke-width="3" stroke-linejoin="round"/>'
        )
        final = rows[-1]
        svg.append(
            f'<circle cx="{sx_ground(final["x_m"]):.1f}" '
            f'cy="{sy_ground(final["y_m"]):.1f}" r="5" '
            f'fill="{COLORS[name]}" stroke="#ffffff" stroke-width="1.5"/>'
        )
    text(
        svg,
        gx + gw / 2,
        ground_y + ground_h - 13,
        "inertial x (m)",
        12,
        "#334155",
        600,
        "middle",
    )
    text(
        svg,
        ground_x + 19,
        gy + gh / 2,
        "inertial y (m)",
        12,
        "#334155",
        600,
        "middle",
    )
    svg[-1] = svg[-1].replace(
        'text-anchor="middle"',
        f'text-anchor="middle" transform="rotate(-90 {ground_x+19},{gy+gh/2})"',
    )

    altitude_x, altitude_y, altitude_w, altitude_h = 714, 148, 674, 212
    panel(
        svg,
        altitude_x,
        altitude_y,
        altitude_w,
        altitude_h,
        "B. Vertical energy management",
        "Altitude history exposes braking, approach, flare, and terminal descent",
    )
    ax, ay = altitude_x + 55, altitude_y + 66
    aw, ah = altitude_w - 82, altitude_h - 94
    max_time = max(rows[-1]["time_s"] for rows in histories.values())
    max_altitude = max(row["z_m"] for rows in histories.values() for row in rows)
    sx_time = lambda value: ax + value / max_time * aw
    sy_altitude = lambda value: ay + ah - value / max_altitude * ah
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        px = sx_time(max_time * fraction)
        py = sy_altitude(max_altitude * fraction)
        svg.append(
            f'<line x1="{px:.1f}" y1="{ay}" x2="{px:.1f}" '
            f'y2="{ay+ah}" stroke="#f1f5f9"/>'
        )
        svg.append(
            f'<line x1="{ax}" y1="{py:.1f}" x2="{ax+aw}" '
            f'y2="{py:.1f}" stroke="#e2e8f0"/>'
        )
        text(svg, px, ay + ah + 18, f"{max_time*fraction:.0f}", 10, "#64748b", anchor="middle")
        text(svg, ax - 9, py + 4, f"{max_altitude*fraction:.0f}", 10, "#64748b", anchor="end")
    for name, rows in histories.items():
        svg.append(
            f'<path d="{line_path(rows, lambda r:r["time_s"], lambda r:r["z_m"], sx_time, sy_altitude)}" '
            f'fill="none" stroke="{COLORS[name]}" stroke-width="2.5"/>'
        )
    text(
        svg,
        altitude_x + 16,
        ay + ah / 2,
        "altitude z (m)",
        11,
        "#334155",
        600,
        "middle",
    )
    svg[-1] = svg[-1].replace(
        'text-anchor="middle"',
        f'text-anchor="middle" transform="rotate(-90 {altitude_x+16},{ay+ah/2})"',
    )

    attitude_x, attitude_y, attitude_w, attitude_h = 714, 388, 674, 212
    panel(
        svg,
        attitude_x,
        attitude_y,
        attitude_w,
        attitude_h,
        "C. Coupled attitude response",
        "Quaternion-derived body-axis tilt; dashed line is the 12 deg terminal criterion",
    )
    tx, ty = attitude_x + 55, attitude_y + 66
    tw, th = attitude_w - 82, attitude_h - 94
    sx_attitude = lambda value: tx + value / max_time * tw
    sy_tilt = lambda value: ty + th - value / 12.5 * th
    for value in (0.0, 3.0, 6.0, 9.0, 12.0):
        py = sy_tilt(value)
        svg.append(
            f'<line x1="{tx}" y1="{py:.1f}" x2="{tx+tw}" '
            f'y2="{py:.1f}" stroke="#e2e8f0"/>'
        )
        text(svg, tx - 9, py + 4, f"{value:.0f}", 10, "#64748b", anchor="end")
    svg.append(
        f'<line x1="{tx}" y1="{sy_tilt(12):.1f}" x2="{tx+tw}" '
        f'y2="{sy_tilt(12):.1f}" stroke="#0f172a" stroke-width="1.5" '
        'stroke-dasharray="6 5"/>'
    )
    for name, rows in histories.items():
        svg.append(
            f'<path d="{line_path(rows, lambda r:r["time_s"], lambda r:r["tilt_deg"], sx_attitude, sy_tilt)}" '
            f'fill="none" stroke="{COLORS[name]}" stroke-width="2.5"/>'
        )
    text(
        svg,
        attitude_x + 16,
        ty + th / 2,
        "tilt (deg)",
        11,
        "#334155",
        600,
        "middle",
    )
    svg[-1] = svg[-1].replace(
        'text-anchor="middle"',
        f'text-anchor="middle" transform="rotate(-90 {attitude_x+16},{ty+th/2})"',
    )

    residual_x, residual_y, residual_w, residual_h = 52, 650, 610, 246
    panel(
        svg,
        residual_x,
        residual_y,
        residual_w,
        residual_h,
        "D. Wrench realization",
        "p95 normalized error: static allocation versus delayed finite-bandwidth actuator tracking",
    )
    rx, ry = residual_x + 64, residual_y + 76
    rw, rh = residual_w - 90, residual_h - 118
    group_width = rw / len(histories)
    residual_max = 0.5
    for index, name in enumerate(histories):
        metrics = summary["cases"][name]["metrics"]
        allocation = metrics["p95_allocation_residual"]
        tracking = metrics[
            "p95_actuator_tracking_residual_after_1s"
        ]
        center = rx + group_width * (index + 0.5)
        for offset, value, color in (
            (-18, allocation, "#0ea5e9"),
            (18, tracking, "#8b5cf6"),
        ):
            bar_height = min(value, residual_max) / residual_max * rh
            svg.append(
                f'<rect x="{center+offset-13:.1f}" y="{ry+rh-bar_height:.1f}" '
                f'width="26" height="{bar_height:.1f}" fill="{color}"/>'
            )
            text(
                svg,
                center + offset,
                ry + rh - bar_height - 7,
                f"{value:.3f}",
                9,
                "#334155",
                600,
                "middle",
            )
        text(svg, center, ry + rh + 19, name.replace("_", " "), 9.5, "#475569", 600, "middle")
    svg.append(
        f'<rect x="{rx+6}" y="{residual_y+56}" width="12" height="12" fill="#0ea5e9"/>'
    )
    text(svg, rx + 24, residual_y + 67, "allocator", 10, "#475569", 600)
    svg.append(
        f'<rect x="{rx+96}" y="{residual_y+56}" width="12" height="12" fill="#8b5cf6"/>'
    )
    text(svg, rx + 114, residual_y + 67, "actuator path", 10, "#475569", 600)

    audit_x, audit_y, audit_w, audit_h = 714, 650, 674, 246
    panel(
        svg,
        audit_x,
        audit_y,
        audit_w,
        audit_h,
        "E. Terminal constraint audit",
        "PASS requires error < 3 m, |vxy| < 1 m/s, |vz| < 2.5 m/s, tilt < 12 deg, and propellant > 0",
    )
    columns = [audit_x + 18, audit_x + 238, audit_x + 338, audit_x + 445, audit_x + 545]
    headers = ["case", "error", "|vz|", "max tilt", "result"]
    for x, header in zip(columns, headers):
        text(svg, x, audit_y + 79, header, 10, "#64748b", 700)
    row_y = audit_y + 105
    for name in histories:
        metrics = summary["cases"][name]["metrics"]
        passed = metrics["success"]
        text(svg, columns[0], row_y, LABELS[name], 11, "#0f172a", 700)
        text(svg, columns[1], row_y, f"{metrics['horizontal_target_error_m']:.2f} m", 11)
        text(svg, columns[2], row_y, f"{metrics['touchdown_vertical_speed_mps']:.2f} m/s", 11)
        text(svg, columns[3], row_y, f"{metrics['maximum_tilt_deg']:.2f} deg", 11)
        text(
            svg,
            columns[4],
            row_y,
            "PASS" if passed else "FAIL",
            11,
            "#059669" if passed else "#dc2626",
            700,
        )
        row_y += 30

    interpretations = [
        (
            52,
            "Force and trajectory coupling",
            [
                "Crosswind changes air-relative velocity and therefore both drag",
                "and normal force through qbar = rho |Vrel|^2 / 2. The vehicle",
                "tilts to create opposing lateral thrust, so position and attitude",
                "cannot be evaluated as independent control channels.",
            ],
        ),
        (
            510,
            "Six-axis control allocation",
            [
                "Axial-thrust imbalance and lateral gimbal force generate body-x/y",
                "moments through the cluster radius and longitudinal CM arm.",
                "Differential tangential gimbal force supplies body-z roll moment.",
                "Low nominal allocation residual confirms wrench attainability.",
            ],
        ),
        (
            968,
            "Two distinct failure mechanisms",
            [
                "High wind exposes steady disturbance error because the scheduled",
                "PD law has no integral or wind-feedforward channel. Engine-out is",
                "different: the asymmetric three-engine feasible wrench set cannot",
                "track the requested force/moment before the vehicle reaches z=0.",
            ],
        ),
    ]
    for x, heading, lines in interpretations:
        svg.append(
            f'<rect x="{x}" y="950" width="420" height="174" rx="4" '
            'fill="#ffffff" stroke="#cbd5e1"/>'
        )
        text(svg, x + 18, 980, heading, 15, "#0f172a", 700)
        for index, line in enumerate(lines):
            text(svg, x + 18, 1010 + 22 * index, line, 11.5, "#334155")
    text(
        svg,
        52,
        1160,
        (
            "Interpretation is model-bounded: this evidence verifies the implemented "
            "rigid-body and actuator equations, not flight qualification."
        ),
        11,
        "#64748b",
    )
    svg.append("</svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
