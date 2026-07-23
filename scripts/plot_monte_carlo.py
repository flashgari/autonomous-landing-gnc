#!/usr/bin/env python3
"""Generate SVG Monte Carlo landing robustness plots."""

import csv
import json
from pathlib import Path


def load_rows(path):
    with path.open() as f:
        rows = []
        for row in csv.DictReader(f):
            parsed = {}
            for key, value in row.items():
                if key in ("success",):
                    parsed[key] = value == "True"
                elif key in ("failure_mode", "guidance_mode"):
                    parsed[key] = value
                else:
                    parsed[key] = float(value)
            rows.append(parsed)
        return rows


def main():
    rows = load_rows(Path("outputs/monte_carlo_landing.csv"))
    summary = json.loads(Path("outputs/monte_carlo_summary.json").read_text())
    write_svg(rows, summary, Path("figures/monte_carlo_landing_dispersion.svg"))
    print("Wrote figures/monte_carlo_landing_dispersion.svg")


def write_svg(rows, summary, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 980, 620
    plot_x, plot_y, plot_w, plot_h = 80, 92, 560, 420
    x_values = [r["landing_x_error_m"] for r in rows]
    v_values = [r["touchdown_vz_mps"] for r in rows]
    x_abs = max(8.0, max(abs(x) for x in x_values) * 1.08)
    v_abs = max(5.0, max(abs(v) for v in v_values) * 1.08)

    def sx(x):
        return plot_x + (x + x_abs) / (2 * x_abs) * plot_w

    def sy(v):
        return plot_y + (v_abs - v) / (2 * v_abs) * plot_h

    modes = summary["failure_modes"]
    mode_items = sorted(modes.items(), key=lambda item: (-item[1], item[0]))
    max_mode = max(modes.values())

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Monte Carlo powered landing dispersion summary</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="52" y="42" font-family="Arial" font-size="24" font-weight="700" fill="#0f172a">Monte Carlo Landing Robustness</text>',
        f'<text x="52" y="66" font-family="Arial" font-size="14" fill="#475569">{summary["num_cases"]} randomized dispersions, seed {summary["seed"]}. Success rate: {summary["success_rate"]*100:.1f}%.</text>',
        f'<rect x="{plot_x}" y="{plot_y}" width="{plot_w}" height="{plot_h}" fill="#ffffff" stroke="#cbd5e1"/>',
    ]
    for frac in [0.25, 0.5, 0.75]:
        x = plot_x + frac * plot_w
        y = plot_y + frac * plot_h
        svg.append(f'<line x1="{x:.1f}" y1="{plot_y}" x2="{x:.1f}" y2="{plot_y+plot_h}" stroke="#e2e8f0"/>')
        svg.append(f'<line x1="{plot_x}" y1="{y:.1f}" x2="{plot_x+plot_w}" y2="{y:.1f}" stroke="#e2e8f0"/>')
    svg.append(f'<rect x="{sx(-3):.1f}" y="{sy(2.5):.1f}" width="{sx(3)-sx(-3):.1f}" height="{sy(-2.5)-sy(2.5):.1f}" fill="#dcfce7" stroke="#16a34a" opacity="0.72"/>')
    svg.append(f'<text x="{sx(-2.8):.1f}" y="{sy(2.5)-8:.1f}" font-family="Arial" font-size="12" fill="#166534">success corridor</text>')
    svg.append(f'<line x1="{sx(0):.1f}" y1="{plot_y}" x2="{sx(0):.1f}" y2="{plot_y+plot_h}" stroke="#94a3b8" stroke-dasharray="5 5"/>')
    svg.append(f'<line x1="{plot_x}" y1="{sy(0):.1f}" x2="{plot_x+plot_w}" y2="{sy(0):.1f}" stroke="#94a3b8" stroke-dasharray="5 5"/>')
    for row in rows:
        color = "#059669" if row["success"] else "#dc2626"
        opacity = "0.82" if row["success"] else "0.62"
        svg.append(f'<circle cx="{sx(row["landing_x_error_m"]):.1f}" cy="{sy(row["touchdown_vz_mps"]):.1f}" r="3.2" fill="{color}" opacity="{opacity}"/>')
    svg.append(f'<text x="{plot_x+plot_w/2-54:.1f}" y="{plot_y+plot_h+42}" font-family="Arial" font-size="14" fill="#334155">landing x error (m)</text>')
    svg.append(f'<text x="20" y="{plot_y+plot_h/2+58:.1f}" font-family="Arial" font-size="14" fill="#334155" transform="rotate(-90 20,{plot_y+plot_h/2+58:.1f})">touchdown vertical velocity (m/s)</text>')
    svg.append(f'<text x="{plot_x}" y="{plot_y+plot_h+20}" font-family="Arial" font-size="12" fill="#64748b">-{x_abs:.1f}</text>')
    svg.append(f'<text x="{plot_x+plot_w-30}" y="{plot_y+plot_h+20}" font-family="Arial" font-size="12" fill="#64748b">{x_abs:.1f}</text>')
    svg.append(f'<text x="{plot_x-56}" y="{plot_y+12}" font-family="Arial" font-size="12" fill="#64748b">{v_abs:.1f}</text>')
    svg.append(f'<text x="{plot_x-60}" y="{plot_y+plot_h}" font-family="Arial" font-size="12" fill="#64748b">-{v_abs:.1f}</text>')

    side_x = 690
    svg.append(f'<text x="{side_x}" y="112" font-family="Arial" font-size="18" font-weight="700" fill="#0f172a">Summary</text>')
    stat_lines = [
        ("success rate", f'{summary["success_rate"]*100:.1f}%'),
        ("p95 |landing error|", f'{summary["p95_abs_landing_error_m"]:.2f} m'),
        ("p95 touchdown speed", f'{summary["p95_touchdown_speed_mps"]:.2f} m/s'),
        ("min prop remaining", f'{summary["min_propellant_remaining_kg"]:.0f} kg'),
        ("max tilt", f'{summary["max_abs_tilt_deg"]:.2f} deg'),
        ("max gimbal", f'{summary["max_abs_gimbal_deg"]:.2f} deg'),
    ]
    y = 142
    for label, value in stat_lines:
        svg.append(f'<text x="{side_x}" y="{y}" font-family="Arial" font-size="13" fill="#64748b">{label}</text>')
        svg.append(f'<text x="{side_x+170}" y="{y}" font-family="Arial" font-size="15" font-weight="700" fill="#0f172a">{value}</text>')
        y += 28
    svg.append(f'<text x="{side_x}" y="{y+24}" font-family="Arial" font-size="18" font-weight="700" fill="#0f172a">Failure modes</text>')
    y += 50
    for mode, count in mode_items:
        bar_w = 190 * count / max_mode
        color = "#059669" if mode == "success" else "#dc2626"
        svg.append(f'<text x="{side_x}" y="{y}" font-family="Arial" font-size="13" fill="#334155">{mode}</text>')
        svg.append(f'<rect x="{side_x}" y="{y+6}" width="{bar_w:.1f}" height="12" fill="{color}" opacity="0.82"/>')
        svg.append(f'<text x="{side_x+bar_w+8:.1f}" y="{y+17}" font-family="Arial" font-size="12" fill="#475569">{count}</text>')
        y += 38
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
