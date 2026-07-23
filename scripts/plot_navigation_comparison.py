#!/usr/bin/env python3
"""Plot estimator error histories and truth-versus-estimated robustness."""

import csv
import json
from pathlib import Path


def load_rows(path):
    with path.open() as f:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(f)]


def svg_path(rows, x_key, y_key, sx, sy):
    return " ".join(
        ("M" if index == 0 else "L") + f"{sx(row[x_key]):.1f},{sy(row[y_key]):.1f}"
        for index, row in enumerate(rows)
    )


def main():
    rows = load_rows(Path("outputs/nominal_landing_estimated.csv"))
    comparison = json.loads(Path("outputs/navigation_comparison.json").read_text())
    write_svg(rows, comparison, Path("figures/navigation_estimation_comparison.svg"))
    print("Wrote figures/navigation_estimation_comparison.svg")


def write_svg(rows, comparison, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 1040, 830
    left, right = 92, 44
    panel_w = width - left - right
    panel_h = 118
    panel_gap = 54
    top = 112
    t_end = rows[-1]["time_s"]
    panels = [
        ("Position estimation error", [("x error", "x_estimation_error_m", "#2563eb"), ("z error", "z_estimation_error_m", "#dc2626")], "m"),
        ("Velocity estimation error", [("vx error", "vx_estimation_error_mps", "#7c3aed"), ("vz error", "vz_estimation_error_mps", "#059669")], "m/s"),
        ("Attitude estimation error", [("pitch error", "theta_estimation_error_deg", "#ea580c")], "deg"),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Navigation estimation and closed-loop robustness comparison</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="52" y="42" font-family="Arial" font-size="24" font-weight="700" fill="#0f172a">Navigation Estimation in the Landing Loop</text>',
        '<text x="52" y="67" font-family="Arial" font-size="14" fill="#475569">Alpha-beta state estimates drive corridor guidance through flight-like throttle and TVC actuators.</text>',
    ]
    for panel_index, (title, series, unit) in enumerate(panels):
        y0 = top + panel_index * (panel_h + panel_gap)
        values = [row[key] for _, key, _ in series for row in rows]
        amplitude = max(0.08, max(abs(value) for value in values) * 1.15)

        def sx(time_s):
            return left + time_s / t_end * panel_w

        def sy(value):
            return y0 + panel_h / 2.0 - value / amplitude * panel_h / 2.0

        svg.append(f'<text x="{left}" y="{y0-12}" font-family="Arial" font-size="16" font-weight="700" fill="#0f172a">{title}</text>')
        svg.append(f'<rect x="{left}" y="{y0}" width="{panel_w}" height="{panel_h}" fill="#ffffff" stroke="#cbd5e1"/>')
        svg.append(f'<line x1="{left}" y1="{sy(0):.1f}" x2="{left+panel_w}" y2="{sy(0):.1f}" stroke="#94a3b8"/>')
        svg.append(f'<text x="24" y="{y0+panel_h/2}" font-family="Arial" font-size="13" fill="#334155" transform="rotate(-90 24,{y0+panel_h/2})">{unit}</text>')
        svg.append(f'<text x="{left-56}" y="{y0+13}" font-family="Arial" font-size="12" fill="#64748b">{amplitude:.2f}</text>')
        svg.append(f'<text x="{left-60}" y="{y0+panel_h}" font-family="Arial" font-size="12" fill="#64748b">-{amplitude:.2f}</text>')
        legend_x = left + 12
        for label, key, color in series:
            svg.append(f'<path d="{svg_path(rows, "time_s", key, sx, sy)}" fill="none" stroke="{color}" stroke-width="2.1"/>')
            svg.append(f'<line x1="{legend_x}" y1="{y0+18}" x2="{legend_x+22}" y2="{y0+18}" stroke="{color}" stroke-width="3"/>')
            svg.append(f'<text x="{legend_x+28}" y="{y0+22}" font-family="Arial" font-size="12" fill="#334155">{label}</text>')
            legend_x += 128

    chart_y = top + 3 * (panel_h + panel_gap) + 12
    truth = comparison["summaries"]["truth"]
    estimated = comparison["summaries"]["estimated"]
    metrics = [
        ("success rate", 100 * truth["success_rate"], 100 * estimated["success_rate"], "%"),
        ("p95 landing error", truth["p95_abs_landing_error_m"], estimated["p95_abs_landing_error_m"], "m"),
        ("p95 touchdown speed", truth["p95_touchdown_speed_mps"], estimated["p95_touchdown_speed_mps"], "m/s"),
    ]
    svg.append(f'<text x="52" y="{chart_y-20}" font-family="Arial" font-size="18" font-weight="700" fill="#0f172a">Closed-loop Monte Carlo consequence</text>')
    group_w = 290
    for index, (label, truth_value, est_value, unit) in enumerate(metrics):
        x0 = 72 + index * 320
        scale = 122.0 / max(truth_value, est_value, 1.0)
        svg.append(f'<text x="{x0}" y="{chart_y+4}" font-family="Arial" font-size="14" font-weight="700" fill="#0f172a">{label}</text>')
        svg.append(f'<rect x="{x0}" y="{chart_y+20}" width="{truth_value*scale:.1f}" height="22" fill="#2563eb" opacity="0.82"/>')
        svg.append(f'<text x="{x0+truth_value*scale+7:.1f}" y="{chart_y+36}" font-family="Arial" font-size="12" fill="#334155">truth {truth_value:.2f} {unit}</text>')
        svg.append(f'<rect x="{x0}" y="{chart_y+52}" width="{est_value*scale:.1f}" height="22" fill="#ea580c" opacity="0.84"/>')
        svg.append(f'<text x="{x0+est_value*scale+7:.1f}" y="{chart_y+68}" font-family="Arial" font-size="12" fill="#334155">estimated {est_value:.2f} {unit}</text>')
    nominal = comparison["nominal_estimation_metrics"]
    svg.append(f'<text x="52" y="{height-30}" font-family="Arial" font-size="13" fill="#475569">Nominal RMS errors: x {nominal["rms_x_estimation_error_m"]:.2f} m, z {nominal["rms_z_estimation_error_m"]:.2f} m, vx {nominal["rms_vx_estimation_error_mps"]:.2f} m/s, vz {nominal["rms_vz_estimation_error_mps"]:.2f} m/s, pitch {nominal["rms_theta_estimation_error_deg"]:.3f} deg.</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
