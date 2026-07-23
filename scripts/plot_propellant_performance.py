#!/usr/bin/env python3
"""Plot propellant use across controlled lateral-target changes."""

import csv
from pathlib import Path


def main():
    with Path("outputs/divert_cost_sweep.csv").open() as f:
        rows = list(csv.DictReader(f))
    write_svg(rows, Path("figures/propellant_performance.svg"))
    print("Wrote figures/propellant_performance.svg")


def write_svg(rows, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 960, 520
    chart_x, chart_y, chart_w, chart_h = 90, 112, 790, 280
    used_values = [float(row["propellant_used_kg"]) for row in rows]
    y_min = min(used_values) - 3.0
    y_max = max(used_values) + 3.0
    slot_w = chart_w / len(rows)

    def sy(value):
        return chart_y + chart_h - (value - y_min) / (y_max - y_min) * chart_h

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Propellant performance across lateral divert targets</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="52" y="42" font-family="Arial" font-size="24" font-weight="700" fill="#0f172a">Divert Demand and Propellant Performance</text>',
        '<text x="52" y="67" font-family="Arial" font-size="14" fill="#475569">Same initial state, navigation seed, guidance, and actuators; only the target coordinate changes.</text>',
        f'<rect x="{chart_x}" y="{chart_y}" width="{chart_w}" height="{chart_h}" fill="#ffffff" stroke="#cbd5e1"/>',
        f'<text x="{chart_x-64}" y="{chart_y+12}" font-family="Arial" font-size="12" fill="#64748b">{y_max:.0f}</text>',
        f'<text x="{chart_x-64}" y="{chart_y+chart_h}" font-family="Arial" font-size="12" fill="#64748b">{y_min:.0f}</text>',
    ]
    for index, row in enumerate(rows):
        target = float(row["target_x_m"])
        correction = float(row["required_lateral_correction_m"])
        used = float(row["propellant_used_kg"])
        success = row["success"] == "True"
        x = chart_x + index * slot_w + 34
        bar_w = slot_w - 68
        bar_y = sy(used)
        fill = "#059669" if success else "#dc2626"
        svg.append(f'<rect x="{x:.1f}" y="{bar_y:.1f}" width="{bar_w:.1f}" height="{chart_y+chart_h-bar_y:.1f}" fill="{fill}" opacity="0.84"/>')
        svg.append(f'<text x="{x+bar_w/2:.1f}" y="{bar_y-10:.1f}" text-anchor="middle" font-family="Arial" font-size="13" font-weight="700" fill="#0f172a">{used:.1f} kg</text>')
        svg.append(f'<text x="{x+bar_w/2:.1f}" y="{chart_y+chart_h+25}" text-anchor="middle" font-family="Arial" font-size="13" font-weight="700" fill="#334155">target {target:+.0f} m</text>')
        svg.append(f'<text x="{x+bar_w/2:.1f}" y="{chart_y+chart_h+44}" text-anchor="middle" font-family="Arial" font-size="12" fill="#64748b">correction {correction:.0f} m</text>')
        svg.append(f'<text x="{x+bar_w/2:.1f}" y="{chart_y+chart_h+63}" text-anchor="middle" font-family="Arial" font-size="12" fill="{fill}">{"PASS" if success else "FAIL"}</text>')
    svg.append(f'<text x="24" y="{chart_y+chart_h/2+58}" font-family="Arial" font-size="14" fill="#334155" transform="rotate(-90 24,{chart_y+chart_h/2+58})">propellant used (kg; truncated scale)</text>')
    svg.append(f'<text x="52" y="{height-32}" font-family="Arial" font-size="13" fill="#475569">The truncated axis exposes a less-than-4 kg spread. The 30 m correction fails on target error despite nearly unchanged total propellant use.</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
