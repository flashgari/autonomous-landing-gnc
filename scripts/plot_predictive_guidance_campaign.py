#!/usr/bin/env python3
"""Create constrained-guidance comparison and active-constraint figures."""

import csv
import json
import math
from pathlib import Path


NAVY = "#0f172a"
TEXT = "#334155"
MUTED = "#64748b"
GRID = "#cbd5e1"
BLUE = "#2563eb"
GREEN = "#059669"
RED = "#dc2626"
PURPLE = "#7c3aed"
ORANGE = "#ea580c"
TEAL = "#0891b2"


def load_rows(path: Path) -> list[dict[str, float]]:
    with path.open() as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def line_path(rows, x_values, y_values, sx, sy) -> str:
    commands = []
    for index, (x_value, y_value) in enumerate(zip(x_values, y_values)):
        if not (math.isfinite(x_value) and math.isfinite(y_value)):
            continue
        commands.append(
            ("M" if not commands else "L")
            + f"{sx(x_value):.1f},{sy(y_value):.1f}"
        )
    return " ".join(commands)


def text_block(
    svg,
    x,
    y,
    lines,
    size=14,
    color=TEXT,
    line_height=21,
    weight=400,
):
    for index, line in enumerate(lines):
        svg.append(
            f'<text x="{x}" y="{y + index * line_height}" '
            f'font-family="Arial" font-size="{size}" font-weight="{weight}" '
            f'fill="{color}">{line}</text>'
        )


def main() -> None:
    corridor = load_rows(Path("outputs/corridor_ekf_nominal.csv"))
    predictive = load_rows(Path("outputs/predictive_ekf_nominal.csv"))
    large_divert = load_rows(Path("outputs/predictive_48m_divert.csv"))
    campaign = json.loads(
        Path("outputs/predictive_guidance_campaign.json").read_text()
    )
    write_comparison(
        corridor,
        predictive,
        campaign,
        Path("figures/predictive_guidance_comparison.svg"),
    )
    write_constraint_activity(
        large_divert,
        campaign,
        Path("figures/predictive_constraint_activity.svg"),
    )
    print("Wrote figures/predictive_guidance_comparison.svg")
    print("Wrote figures/predictive_constraint_activity.svg")


def write_comparison(corridor, predictive, campaign, path: Path) -> None:
    width, height = 1500, 1040
    chart_left = 90
    chart_top = 145
    chart_width = 740
    chart_height = 355
    side_x = 900
    corridor_summary = campaign["monte_carlo_summaries"]["corridor"]
    predictive_summary = campaign["monte_carlo_summaries"]["predictive"]
    delta = campaign["delta_predictive_minus_corridor"]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Constrained predictive guidance compared with corridor guidance</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="62" y="52" font-family="Arial" font-size="31" font-weight="700" fill="{NAVY}">Constrained Powered-Descent Guidance</text>',
        f'<text x="62" y="82" font-family="Arial" font-size="16" fill="{MUTED}">ESKF feedback and flight-like actuators held fixed; only the high-altitude guidance architecture changes.</text>',
        f'<text x="{chart_left}" y="{chart_top-18}" font-family="Arial" font-size="20" font-weight="700" fill="{NAVY}">Nominal downrange-altitude trajectory</text>',
        f'<rect x="{chart_left}" y="{chart_top}" width="{chart_width}" height="{chart_height}" fill="#ffffff" stroke="{GRID}"/>',
    ]
    all_x = [row["x_m"] for row in corridor + predictive]
    all_z = [row["z_m"] for row in corridor + predictive]
    x_min = min(all_x) - 1.5
    x_max = max(all_x) + 1.5
    z_max = max(all_z)

    def sx(value):
        return chart_left + (value - x_min) / (x_max - x_min) * chart_width

    def sy(value):
        return chart_top + chart_height - value / z_max * chart_height

    for fraction in (0.25, 0.50, 0.75):
        y = chart_top + chart_height * (1.0 - fraction)
        svg.append(
            f'<line x1="{chart_left}" y1="{y:.1f}" x2="{chart_left+chart_width}" y2="{y:.1f}" stroke="#e2e8f0"/>'
        )
        svg.append(
            f'<text x="{chart_left-48}" y="{y+4:.1f}" font-family="Arial" font-size="12" fill="{MUTED}">{z_max*fraction:.0f}</text>'
        )
    svg.extend(
        [
            f'<path d="{line_path(corridor, [row["x_m"] for row in corridor], [row["z_m"] for row in corridor], sx, sy)}" fill="none" stroke="{ORANGE}" stroke-width="3"/>',
            f'<path d="{line_path(predictive, [row["x_m"] for row in predictive], [row["z_m"] for row in predictive], sx, sy)}" fill="none" stroke="{TEAL}" stroke-width="3"/>',
            f'<line x1="{chart_left+18}" y1="{chart_top+22}" x2="{chart_left+47}" y2="{chart_top+22}" stroke="{ORANGE}" stroke-width="4"/>',
            f'<text x="{chart_left+56}" y="{chart_top+27}" font-family="Arial" font-size="13" fill="{TEXT}">corridor</text>',
            f'<line x1="{chart_left+145}" y1="{chart_top+22}" x2="{chart_left+174}" y2="{chart_top+22}" stroke="{TEAL}" stroke-width="4"/>',
            f'<text x="{chart_left+183}" y="{chart_top+27}" font-family="Arial" font-size="13" fill="{TEXT}">predictive</text>',
            f'<text x="{chart_left-65}" y="{chart_top+chart_height/2}" transform="rotate(-90 {chart_left-65},{chart_top+chart_height/2})" font-family="Arial" font-size="14" fill="{TEXT}">altitude z (m)</text>',
            f'<text x="{chart_left+chart_width/2-55}" y="{chart_top+chart_height+35}" font-family="Arial" font-size="14" fill="{TEXT}">downrange x (m)</text>',
        ]
    )
    for fraction in (0.0, 0.5, 1.0):
        value = x_min + fraction * (x_max - x_min)
        x = sx(value)
        svg.append(
            f'<text x="{x-13:.1f}" y="{chart_top+chart_height+18}" font-family="Arial" font-size="11" fill="{MUTED}">{value:.0f}</text>'
        )

    text_block(
        svg,
        side_x,
        132,
        ["Matched 200-case result"],
        size=21,
        color=NAVY,
        weight=700,
    )
    result_lines = [
        (
            "success",
            f'{100*corridor_summary["success_rate"]:.1f}%',
            f'{100*predictive_summary["success_rate"]:.1f}%',
        ),
        (
            "p95 |pad error|",
            f'{corridor_summary["p95_abs_landing_error_m"]:.2f} m',
            f'{predictive_summary["p95_abs_landing_error_m"]:.2f} m',
        ),
        (
            "p95 touchdown speed",
            f'{corridor_summary["p95_touchdown_speed_mps"]:.2f} m/s',
            f'{predictive_summary["p95_touchdown_speed_mps"]:.2f} m/s',
        ),
        (
            "minimum propellant",
            f'{corridor_summary["min_propellant_remaining_kg"]:.0f} kg',
            f'{predictive_summary["min_propellant_remaining_kg"]:.0f} kg',
        ),
    ]
    svg.extend(
        [
            f'<text x="{side_x+205}" y="170" font-family="Arial" font-size="13" font-weight="700" fill="{ORANGE}">corridor</text>',
            f'<text x="{side_x+340}" y="170" font-family="Arial" font-size="13" font-weight="700" fill="{TEAL}">predictive</text>',
        ]
    )
    for index, (label, base, advanced) in enumerate(result_lines):
        y = 204 + index * 42
        svg.extend(
            [
                f'<text x="{side_x}" y="{y}" font-family="Arial" font-size="14" fill="{TEXT}">{label}</text>',
                f'<text x="{side_x+205}" y="{y}" font-family="Arial" font-size="14" fill="{TEXT}">{base}</text>',
                f'<text x="{side_x+340}" y="{y}" font-family="Arial" font-size="14" font-weight="700" fill="{NAVY}">{advanced}</text>',
            ]
        )
    text_block(
        svg,
        side_x,
        390,
        ["Physical interpretation"],
        size=19,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        side_x,
        425,
        [
            f"Success increases by {delta['success_rate_points']:+.1f} points",
            f"and p95 pad error changes by {delta['p95_abs_landing_error_m']:+.2f} m.",
            "The predictive trajectory removes crossrange",
            "velocity earlier, while altitude and time-to-go",
            "are large. The 160 m terminal handoff then",
            "reserves the final phase for braking and",
            "actuator-lag accommodation.",
        ],
    )

    sweep_top = 615
    sweep_left = 90
    sweep_width = 850
    sweep_height = 280
    sweep = campaign["reachability_sweep"]
    svg.extend(
        [
            f'<line x1="62" y1="565" x2="{width-62}" y2="565" stroke="{GRID}" stroke-width="2"/>',
            f'<text x="{sweep_left}" y="{sweep_top-20}" font-family="Arial" font-size="20" font-weight="700" fill="{NAVY}">Deterministic lateral-reachability sweep</text>',
            f'<rect x="{sweep_left}" y="{sweep_top}" width="{sweep_width}" height="{sweep_height}" fill="#ffffff" stroke="{GRID}"/>',
        ]
    )
    x_values = sorted({float(row["initial_x_m"]) for row in sweep})
    error_limit = 8.0

    def sx_sweep(value):
        return sweep_left + value / max(x_values) * sweep_width

    def sy_sweep(value):
        clipped = max(-error_limit, min(error_limit, value))
        return sweep_top + sweep_height / 2.0 - clipped / error_limit * sweep_height / 2.0

    for limit in (-3.0, 0.0, 3.0):
        y = sy_sweep(limit)
        color = RED if abs(limit) == 3.0 else "#94a3b8"
        dash = ' stroke-dasharray="7 5"' if abs(limit) == 3.0 else ""
        svg.append(
            f'<line x1="{sweep_left}" y1="{y:.1f}" x2="{sweep_left+sweep_width}" y2="{y:.1f}" stroke="{color}"{dash}/>'
        )
        svg.append(
            f'<text x="{sweep_left-31}" y="{y+4:.1f}" font-family="Arial" font-size="11" fill="{MUTED}">{limit:+.0f}</text>'
        )
    for value in (0, 20, 40, 60):
        x = sx_sweep(value)
        svg.append(
            f'<text x="{x-8:.1f}" y="{sweep_top+sweep_height+18}" font-family="Arial" font-size="11" fill="{MUTED}">{value}</text>'
        )
    for mode, color in (("corridor", ORANGE), ("predictive", TEAL)):
        rows = sorted(
            (row for row in sweep if row["guidance_mode"] == mode),
            key=lambda row: row["initial_x_m"],
        )
        path_data = line_path(
            rows,
            [float(row["initial_x_m"]) for row in rows],
            [float(row["landing_x_error_m"]) for row in rows],
            sx_sweep,
            sy_sweep,
        )
        svg.append(
            f'<path d="{path_data}" fill="none" stroke="{color}" stroke-width="2.5"/>'
        )
        for row in rows:
            fill = color if row["success"] else "#ffffff"
            svg.append(
                f'<circle cx="{sx_sweep(float(row["initial_x_m"])):.1f}" cy="{sy_sweep(float(row["landing_x_error_m"])):.1f}" r="6" fill="{fill}" stroke="{color}" stroke-width="2"/>'
            )
    svg.extend(
        [
            f'<text x="{sweep_left+sweep_width/2-58}" y="{sweep_top+sweep_height+36}" font-family="Arial" font-size="14" fill="{TEXT}">initial offset (m)</text>',
            f'<text x="{sweep_left-68}" y="{sweep_top+sweep_height/2}" transform="rotate(-90 {sweep_left-68},{sweep_top+sweep_height/2})" font-family="Arial" font-size="14" fill="{TEXT}">landing error (m)</text>',
            f'<text x="{sweep_left+12}" y="{sy_sweep(3)-8:.1f}" font-family="Arial" font-size="12" fill="{RED}">+/-3 m requirement</text>',
        ]
    )
    text_block(
        svg,
        1000,
        610,
        ["Reachability result"],
        size=20,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        1000,
        650,
        [
            "Filled markers pass all touchdown constraints;",
            "open markers fail. Corridor guidance remains",
            "successful through a 30 m initial offset.",
            "Predictive shaping extends that coarse sampled",
            "boundary to 40 m. A separate 48 m case also",
            "passes at the finer 0.05 s integration step.",
            "",
            "Both modes fail at 60-70 m. This retained",
            "failure boundary is physical: finite time,",
            "tilt, thrust, and terminal-velocity requirements",
            "bound the available lateral impulse.",
        ],
    )
    text_block(
        svg,
        90,
        995,
        [
            "The sweep is a sampled closed-loop map, not a formal reachable set; no guarantee is made between tested offsets.",
        ],
        size=13,
        color=MUTED,
    )
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


def write_constraint_activity(rows, campaign, path: Path) -> None:
    width, height = 1500, 1110
    left = 95
    chart_width = 850
    side_x = 1010
    panel_height = 165
    panel_gap = 72
    top = 140
    end_time = rows[-1]["time_s"]
    handoff_time = next(
        row["time_s"] for row in rows if row["optimizer_terminal_handoff"] > 0.5
    )
    predictive_rows = [
        row for row in rows if row["time_s"] <= handoff_time + 1.0e-9
    ]
    deterministic = campaign["deterministic_cases"]["predictive_48m_divert"]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Predictive-guidance constraint activity and solver residuals</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="62" y="52" font-family="Arial" font-size="31" font-weight="700" fill="{NAVY}">Constraint Activity During a 48 m Divert</text>',
        f'<text x="62" y="82" font-family="Arial" font-size="16" fill="{MUTED}">Positive margin is feasible; zero indicates an active constraint. The terminal controller takes over below 160 m.</text>',
    ]
    panel_specs = [
        (
            "Glide-slope margin",
            "optimizer_minimum_glideslope_margin_m",
            "m",
            GREEN,
        ),
        (
            "Tilt-cone margin",
            "optimizer_minimum_tilt_margin_deg",
            "deg",
            PURPLE,
        ),
        (
            "Maximum-thrust margin",
            "optimizer_minimum_thrust_margin_mps2",
            "m/s^2",
            BLUE,
        ),
    ]
    for index, (title, key, unit, color) in enumerate(panel_specs):
        y0 = top + index * (panel_height + panel_gap)
        values = [row[key] for row in predictive_rows]
        minimum = min(0.0, min(values))
        maximum = max(values)
        span = max(0.1, maximum - minimum)
        lower = minimum - 0.08 * span
        upper = maximum + 0.08 * span

        def sx(time_s):
            return left + time_s / end_time * chart_width

        def sy(value):
            return y0 + panel_height - (value - lower) / (upper - lower) * panel_height

        svg.extend(
            [
                f'<text x="{left}" y="{y0-14}" font-family="Arial" font-size="18" font-weight="700" fill="{NAVY}">{title}</text>',
                f'<rect x="{left}" y="{y0}" width="{chart_width}" height="{panel_height}" fill="#ffffff" stroke="{GRID}"/>',
                f'<rect x="{sx(handoff_time):.1f}" y="{y0}" width="{left+chart_width-sx(handoff_time):.1f}" height="{panel_height}" fill="#e2e8f0" opacity="0.55"/>',
                f'<line x1="{left}" y1="{sy(0):.1f}" x2="{left+chart_width}" y2="{sy(0):.1f}" stroke="{RED}" stroke-dasharray="7 5"/>',
                f'<path d="{line_path(predictive_rows, [row["time_s"] for row in predictive_rows], values, sx, sy)}" fill="none" stroke="{color}" stroke-width="2.2"/>',
                f'<line x1="{sx(handoff_time):.1f}" y1="{y0}" x2="{sx(handoff_time):.1f}" y2="{y0+panel_height}" stroke="{NAVY}" stroke-width="1.4"/>',
                f'<text x="{sx(handoff_time)+8:.1f}" y="{y0+20}" font-family="Arial" font-size="12" fill="{NAVY}">terminal handoff</text>',
                f'<text x="{left-60}" y="{y0+14}" font-family="Arial" font-size="12" fill="{MUTED}">{upper:.1f}</text>',
                f'<text x="{left-60}" y="{y0+panel_height}" font-family="Arial" font-size="12" fill="{MUTED}">{lower:.1f}</text>',
                f'<text x="{left+10}" y="{y0+19}" font-family="Arial" font-size="12" fill="{color}">minimum predicted margin [{unit}]</text>',
            ]
        )

    solver_y = top + 3 * (panel_height + panel_gap)
    replan_rows = [row for row in rows if row["optimizer_replanned"] > 0.5]
    residuals = [
        max(
            row["optimizer_primal_residual"],
            row["optimizer_dual_residual"],
            1.0e-5,
        )
        for row in replan_rows
    ]
    log_lower = -5.0
    log_upper = 1.0

    def sx_solver(time_s):
        return left + time_s / end_time * chart_width

    def sy_solver(value):
        log_value = max(log_lower, min(log_upper, math.log10(value)))
        return solver_y + panel_height - (log_value - log_lower) / (
            log_upper - log_lower
        ) * panel_height

    tolerance = 0.025
    svg.extend(
        [
            f'<text x="{left}" y="{solver_y-14}" font-family="Arial" font-size="18" font-weight="700" fill="{NAVY}">ADMM optimality residual at each replan</text>',
            f'<rect x="{left}" y="{solver_y}" width="{chart_width}" height="{panel_height}" fill="#ffffff" stroke="{GRID}"/>',
            f'<line x1="{left}" y1="{sy_solver(tolerance):.1f}" x2="{left+chart_width}" y2="{sy_solver(tolerance):.1f}" stroke="{RED}" stroke-dasharray="7 5"/>',
            f'<path d="{line_path(replan_rows, [row["time_s"] for row in replan_rows], residuals, sx_solver, sy_solver)}" fill="none" stroke="{ORANGE}" stroke-width="2.2"/>',
            f'<text x="{left+chart_width-130}" y="{sy_solver(tolerance)-8:.1f}" font-family="Arial" font-size="12" fill="{RED}">optimality tolerance</text>',
            f'<text x="{left+chart_width/2-25}" y="{solver_y+panel_height+34}" font-family="Arial" font-size="14" fill="{TEXT}">time (s)</text>',
        ]
    )
    for time_s in (0.0, 10.0, 20.0, 30.0, 40.0):
        x = sx_solver(time_s)
        svg.append(
            f'<text x="{x-8:.1f}" y="{solver_y+panel_height+18}" font-family="Arial" font-size="11" fill="{MUTED}">{time_s:.0f}</text>'
        )
    for exponent in (-5, -3, -1, 1):
        y = sy_solver(10.0**exponent)
        svg.append(
            f'<text x="{left-42}" y="{y+4:.1f}" font-family="Arial" font-size="11" fill="{MUTED}">1e{exponent}</text>'
        )

    text_block(
        svg,
        side_x,
        132,
        ["What becomes active"],
        size=20,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        side_x,
        171,
        [
            "The glide-slope margin approaches zero during",
            "the large divert. That means terrain-relative",
            "geometry, rather than thrust or tilt, is the",
            "binding high-altitude path constraint.",
            "",
            "Tilt and thrust margins remain positive, so",
            "the 48 m case is not won by saturating control.",
            "It is won by distributing lateral acceleration",
            "over the available time-to-go.",
        ],
    )
    text_block(
        svg,
        side_x,
        415,
        ["Terminal handoff"],
        size=20,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        side_x,
        454,
        [
            f"The handoff occurs at t = {handoff_time:.1f} s when",
            "the estimated altitude crosses 160 m. Below",
            "that point, actuator lag and minimum-throttle",
            "switching matter more than long-horizon path",
            "shaping, so the verified terminal corridor",
            "controller completes braking and touchdown.",
        ],
    )
    text_block(
        svg,
        side_x,
        650,
        ["Solver interpretation"],
        size=20,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        side_x,
        689,
        [
            "ADMM separates the quadratic objective from",
            "projection onto linear constraint intervals.",
            "A feasible iterate can be accepted before the",
            "dual residual reaches the tighter optimality",
            "tolerance; feasibility and convergence are",
            "therefore reported as separate quantities.",
            "",
            f"This divert lands at {deterministic['landing_x_error_m']:+.2f} m",
            f"with speed {deterministic['touchdown_speed_mps']:.2f} m/s.",
            "The worst QP violation is retained in the data",
            "rather than hidden by the plotting layer.",
        ],
    )
    text_block(
        svg,
        side_x,
        960,
        ["Model boundary"],
        size=20,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        side_x,
        999,
        [
            "This is a planar convex relaxation with a",
            "hybrid throttle supervisor, not a flight-ready",
            "six-DOF successive-convexification solver.",
        ],
    )
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
