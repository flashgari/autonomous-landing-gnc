#!/usr/bin/env python3
"""Generate recruiter-facing 6-DOF NMPC verification evidence."""

from __future__ import annotations

import csv
import html
import json
import math
from pathlib import Path


BASELINE = "#dc2626"
NMPC = "#0f766e"
INK = "#0f172a"
MUTED = "#475569"
GRID = "#e2e8f0"


def load_rows(path: str) -> list[dict[str, float]]:
    with Path(path).open() as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def text(
    svg,
    x,
    y,
    value,
    size=12,
    color=MUTED,
    weight=400,
    anchor="start",
):
    svg.append(
        f'<text x="{x}" y="{y}" text-anchor="{anchor}" '
        'font-family="Arial,Helvetica,sans-serif" '
        f'font-size="{size}" font-weight="{weight}" fill="{color}">'
        f"{html.escape(str(value))}</text>"
    )


def panel(svg, x, y, width, height, title, subtitle):
    svg.append(
        f'<rect x="{x}" y="{y}" width="{width}" height="{height}" '
        'rx="4" fill="#ffffff" stroke="#cbd5e1"/>'
    )
    text(svg, x + 18, y + 30, title, 16, INK, 700)
    text(svg, x + 18, y + 51, subtitle, 11, "#64748b")


def line_path(rows, x_key, y_key, sx, sy):
    return " ".join(
        ("M" if index == 0 else "L")
        + f"{sx(row[x_key]):.1f},{sy(row[y_key]):.1f}"
        for index, row in enumerate(rows)
    )


def main() -> None:
    summary = json.loads(
        Path("outputs/sixdof_nmpc_campaign_summary.json").read_text()
    )
    high = {
        "baseline": load_rows(
            "outputs/sixdof_baseline_high_crosswind_comparison.csv"
        ),
        "nmpc": load_rows(
            "outputs/sixdof_nmpc_high_crosswind_comparison.csv"
        ),
    }
    write_svg(
        summary,
        high,
        Path("figures/sixdof_nmpc_verification.svg"),
    )
    print("Wrote figures/sixdof_nmpc_verification.svg")


def write_svg(summary, histories, path):
    width, height = 1440, 1180
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Six degree of freedom nonlinear MPC verification</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
    ]
    text(
        svg,
        48,
        48,
        "6-DOF Nonlinear MPC Guidance Verification",
        28,
        INK,
        700,
    )
    text(
        svg,
        48,
        76,
        (
            "Reduced-order attitude-aware prediction closed around the "
            "14-state quaternion truth plant and four-engine actuator stack"
        ),
        14,
        MUTED,
    )
    for x, color, label in (
        (48, BASELINE, "scheduled feedback"),
        (250, NMPC, "trust-region NMPC"),
    ):
        svg.append(
            f'<line x1="{x}" y1="101" x2="{x+32}" y2="101" '
            f'stroke="{color}" stroke-width="4"/>'
        )
        text(svg, x + 42, 106, label, 12, MUTED, 600)

    # High-crosswind ground track.
    panel(
        svg,
        48,
        126,
        650,
        410,
        "A. Retained high-crosswind boundary",
        "18/-10 m/s wind; both miss the 3 m footprint, NMPC reduces the miss",
    )
    px, py, pw, ph = 108, 206, 545, 275
    values = [
        row[key]
        for rows in histories.values()
        for row in rows
        for key in ("x_m", "y_m")
    ]
    low = 5.0 * math.floor((min(values) - 2.0) / 5.0)
    high_value = 5.0 * math.ceil((max(values) + 2.0) / 5.0)
    span = max(5.0, high_value - low)
    sx = lambda value: px + (value - low) / span * pw
    sy = lambda value: py + ph - (value - low) / span * ph
    tick = low
    while tick <= high_value + 1.0e-9:
        gx, gy = sx(tick), sy(tick)
        svg.append(
            f'<line x1="{gx:.1f}" y1="{py}" x2="{gx:.1f}" '
            f'y2="{py+ph}" stroke="{GRID}"/>'
        )
        svg.append(
            f'<line x1="{px}" y1="{gy:.1f}" x2="{px+pw}" '
            f'y2="{gy:.1f}" stroke="{GRID}"/>'
        )
        text(svg, gx, py + ph + 19, f"{tick:.0f}", 9, "#64748b", anchor="middle")
        text(svg, px - 9, gy + 4, f"{tick:.0f}", 9, "#64748b", anchor="end")
        tick += 5.0
    svg.append(
        f'<circle cx="{sx(0):.1f}" cy="{sy(0):.1f}" r="9" '
        'fill="none" stroke="#111827" stroke-width="2"/>'
    )
    for mode, rows in histories.items():
        color = BASELINE if mode == "baseline" else NMPC
        svg.append(
            f'<path d="{line_path(rows, "x_m", "y_m", sx, sy)}" '
            f'fill="none" stroke="{color}" stroke-width="3"/>'
        )
        final = rows[-1]
        svg.append(
            f'<circle cx="{sx(final["x_m"]):.1f}" '
            f'cy="{sy(final["y_m"]):.1f}" r="5" fill="{color}" '
            'stroke="#ffffff" stroke-width="1.5"/>'
        )
    text(svg, px + pw / 2, 520, "inertial x (m)", 11, MUTED, 600, "middle")
    text(
        svg,
        70,
        py + ph / 2,
        "inertial y (m)",
        11,
        MUTED,
        600,
        "middle",
    )
    svg[-1] = svg[-1].replace(
        'text-anchor="middle"',
        f'text-anchor="middle" transform="rotate(-90 70,{py+ph/2})"',
    )

    # Vertical-energy history.
    panel(
        svg,
        734,
        126,
        658,
        410,
        "B. Vertical energy schedule",
        "NMPC completes braking sooner without increasing touchdown descent speed",
    )
    ax, ay, aw, ah = 794, 206, 550, 275
    max_time = max(rows[-1]["time_s"] for rows in histories.values())
    max_altitude = max(
        row["z_m"] for rows in histories.values() for row in rows
    )
    st = lambda value: ax + value / max_time * aw
    sz = lambda value: ay + ah - value / max_altitude * ah
    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        gx = st(max_time * fraction)
        gy = sz(max_altitude * fraction)
        svg.append(
            f'<line x1="{gx:.1f}" y1="{ay}" x2="{gx:.1f}" '
            f'y2="{ay+ah}" stroke="{GRID}"/>'
        )
        svg.append(
            f'<line x1="{ax}" y1="{gy:.1f}" x2="{ax+aw}" '
            f'y2="{gy:.1f}" stroke="{GRID}"/>'
        )
        text(svg, gx, ay + ah + 19, f"{max_time*fraction:.0f}", 9, "#64748b", anchor="middle")
        text(svg, ax - 9, gy + 4, f"{max_altitude*fraction:.0f}", 9, "#64748b", anchor="end")
    for mode, rows in histories.items():
        color = BASELINE if mode == "baseline" else NMPC
        svg.append(
            f'<path d="{line_path(rows, "time_s", "z_m", st, sz)}" '
            f'fill="none" stroke="{color}" stroke-width="3"/>'
        )
    text(svg, ax + aw / 2, 520, "time (s)", 11, MUTED, 600, "middle")
    text(svg, 756, ay + ah / 2, "altitude (m)", 11, MUTED, 600, "middle")
    svg[-1] = svg[-1].replace(
        'text-anchor="middle"',
        f'text-anchor="middle" transform="rotate(-90 756,{ay+ah/2})"',
    )

    # Monte Carlo outcomes.
    baseline = summary["baseline"]
    nmpc = summary["nmpc"]
    panel(
        svg,
        48,
        566,
        420,
        300,
        "C. Matched 3D dispersions",
        "24 initial-state, mass, and wind draws; seed 9242",
    )
    metrics = (
        (
            "success rate",
            100.0 * baseline["success_rate"],
            100.0 * nmpc["success_rate"],
            100.0,
            "%",
        ),
        (
            "p95 footprint error",
            baseline["p95_horizontal_error_m"],
            nmpc["p95_horizontal_error_m"],
            5.0,
            "m",
        ),
    )
    y = 650
    for label, base_value, nmpc_value, scale, unit in metrics:
        text(svg, 70, y, label, 12, INK, 700)
        for offset, value, color, name in (
            (20, base_value, BASELINE, "baseline"),
            (54, nmpc_value, NMPC, "NMPC"),
        ):
            bar_width = 245.0 * value / scale
            svg.append(
                f'<rect x="144" y="{y+offset-15}" width="{bar_width:.1f}" '
                f'height="20" fill="{color}" opacity="0.88"/>'
            )
            text(
                svg,
                150 + bar_width,
                y + offset,
                f"{name} {value:.1f}{unit}",
                10,
                MUTED,
                600,
            )
        y += 104

    # Propellant interpretation.
    panel(
        svg,
        492,
        566,
        420,
        300,
        "D. Gravity-loss trade",
        "Median modeled propellant remaining at touchdown",
    )
    base_prop = baseline["median_propellant_remaining_kg"]
    nmpc_prop = nmpc["median_propellant_remaining_kg"]
    max_prop = 4200.0
    for index, (label, value, color) in enumerate(
        (
            ("baseline", base_prop, BASELINE),
            ("NMPC", nmpc_prop, NMPC),
        )
    ):
        y_bar = 655 + index * 66
        svg.append(
            f'<rect x="570" y="{y_bar}" width="{270*value/max_prop:.1f}" '
            f'height="29" fill="{color}" opacity="0.88"/>'
        )
        text(svg, 514, y_bar + 20, label, 11, INK, 700)
        text(svg, 580 + 270 * value / max_prop, y_bar + 20, f"{value:.0f} kg", 11, MUTED, 700)
    prop_delta = nmpc_prop - base_prop
    text(
        svg,
        514,
        809,
        f"+{prop_delta:.0f} kg median reserve",
        18,
        NMPC,
        700,
    )
    text(
        svg,
        514,
        837,
        "from shorter powered descent, not higher terminal speed",
        11,
        MUTED,
    )

    # Timing.
    panel(
        svg,
        936,
        566,
        456,
        300,
        "E. Replan timing audit",
        "Measured on the local Python runtime; deadline is the 0.8 s update period",
    )
    timing = summary["timing"]
    values = (
        ("mean", timing["observed_mean_solve_time_ms"]),
        ("p95", timing["observed_p95_solve_time_ms"]),
        ("maximum", timing["observed_maximum_solve_time_ms"]),
    )
    for index, (label, value) in enumerate(values):
        y_bar = 650 + 54 * index
        bar_width = 310 * value / timing["replan_deadline_ms"]
        text(svg, 958, y_bar + 17, label, 10, INK, 700)
        svg.append(
            f'<rect x="1028" y="{y_bar}" width="{bar_width:.1f}" '
            'height="23" fill="#2563eb"/>'
        )
        text(svg, 1037 + bar_width, y_bar + 17, f"{value:.0f} ms", 10, MUTED, 700)
    svg.append(
        '<line x1="1338" y1="635" x2="1338" y2="812" '
        'stroke="#111827" stroke-width="2" stroke-dasharray="5 4"/>'
    )
    text(svg, 1338, 829, "800 ms", 10, INK, 700, "middle")

    # Physical interpretation band.
    panel(
        svg,
        48,
        898,
        1344,
        232,
        "Physical interpretation",
        "What changed in the closed-loop mechanics, and what did not",
    )
    lines = [
        (
            "1. Horizon coupling: NMPC schedules lateral impulse and counter-impulse "
            "before the terminal phase, preserving vertical thrust projection for braking."
        ),
        (
            "2. Robustness: success rises 70.8% to 79.2% and p95 miss falls 3.95 m to "
            "3.48 m; five footprint failures remain, so the disturbance envelope is finite."
        ),
        (
            "3. Propellant: the roughly 651 kg median reserve increase comes from reducing "
            "powered-descent time and gravity loss while retaining about 1.20 m/s touchdown speed."
        ),
        (
            "4. High wind: the deterministic miss falls 6.16 m to 4.88 m but still fails; "
            "known-wind drag prediction improves feedforward without creating extra tilt authority."
        ),
        (
            "5. Engine out: the supervisor rejects the nominal plan, yet the three-engine "
            "wrench map remains rank-deficient; that case is a retained allocation failure boundary."
        ),
    ]
    for index, line in enumerate(lines):
        text(svg, 72, 980 + 31 * index, line, 12, MUTED, 400)

    svg.append("</svg>")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
