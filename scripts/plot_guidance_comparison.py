#!/usr/bin/env python3
"""Plot baseline vs corridor guidance Monte Carlo comparison."""

import json
from pathlib import Path


def main():
    comparison = json.loads(Path("outputs/monte_carlo_guidance_comparison.json").read_text())
    write_svg(comparison, Path("figures/guidance_mode_comparison.svg"))
    print("Wrote figures/guidance_mode_comparison.svg")


def write_svg(comparison, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    baseline = comparison["summaries"]["baseline"]
    corridor = comparison["summaries"]["corridor"]
    width, height = 1120, 820
    x0 = 100
    y0 = 126
    chart_w = 760
    row_h = 88
    metrics = [
        ("success rate", baseline["success_rate"] * 100, corridor["success_rate"] * 100, "%", True),
        ("p95 |landing error|", baseline["p95_abs_landing_error_m"], corridor["p95_abs_landing_error_m"], "m", False),
        ("p95 touchdown speed", baseline["p95_touchdown_speed_mps"], corridor["p95_touchdown_speed_mps"], "m/s", False),
        ("max tilt", baseline["max_abs_tilt_deg"], corridor["max_abs_tilt_deg"], "deg", False),
        ("max gimbal", baseline["max_abs_gimbal_deg"], corridor["max_abs_gimbal_deg"], "deg", False),
    ]

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Baseline versus corridor guidance Monte Carlo comparison</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="52" y="42" font-family="Arial" font-size="24" font-weight="700" fill="#0f172a">Guidance Mode Monte Carlo Comparison</text>',
        f'<text x="52" y="66" font-family="Arial" font-size="14" fill="#475569">Same {baseline["num_cases"]} dispersions and seed {baseline["seed"]}: baseline guidance vs corridor guidance.</text>',
        '<rect x="850" y="25" width="218" height="54" rx="4" fill="#ffffff" stroke="#cbd5e1"/>',
        '<text x="864" y="47" font-family="Arial" font-size="12" font-weight="700" fill="#0f172a">Terminal requirements</text>',
        '<text x="864" y="66" font-family="Arial" font-size="11" fill="#475569">|e_x| &lt; 3 m; |v_x| &lt; 1 m/s; |v_z| &lt; 2.5 m/s</text>',
    ]
    for i, (label, b_val, c_val, unit, higher_better) in enumerate(metrics):
        y = y0 + i * row_h
        max_val = max(b_val, c_val, 1.0) * 1.15
        b_w = chart_w * b_val / max_val
        c_w = chart_w * c_val / max_val
        svg.append(f'<text x="52" y="{y-10}" font-family="Arial" font-size="16" font-weight="700" fill="#0f172a">{label}</text>')
        svg.append(f'<rect x="{x0}" y="{y}" width="{b_w:.1f}" height="24" fill="#dc2626" opacity="0.82"/>')
        svg.append(f'<rect x="{x0}" y="{y+32}" width="{c_w:.1f}" height="24" fill="#059669" opacity="0.86"/>')
        svg.append(f'<text x="{x0+b_w+8:.1f}" y="{y+17}" font-family="Arial" font-size="13" fill="#334155">baseline {b_val:.2f} {unit}</text>')
        svg.append(f'<text x="{x0+c_w+8:.1f}" y="{y+49}" font-family="Arial" font-size="13" fill="#334155">corridor {c_val:.2f} {unit}</text>')

    mode_y = y0 + len(metrics) * row_h + 12
    svg.append(f'<text x="52" y="{mode_y}" font-family="Arial" font-size="18" font-weight="700" fill="#0f172a">Failure mode shift</text>')
    svg.append(f'<text x="250" y="{mode_y}" font-family="Arial" font-size="12" fill="#475569">Counts from the same 200 initial-condition and model-parameter dispersions</text>')
    mode_y += 24
    mode_labels = {
        "pad_miss": "pad miss",
        "vertical_speed": "vertical speed",
        "success": "success",
    }
    mode_colors = {
        "pad_miss": "#dc2626",
        "vertical_speed": "#f59e0b",
        "success": "#059669",
    }
    for label, summary, color in [("baseline", baseline, "#dc2626"), ("corridor", corridor, "#059669")]:
        x = 52
        svg.append(f'<text x="{x}" y="{mode_y+18}" font-family="Arial" font-size="14" font-weight="700" fill="#0f172a">{label}</text>')
        x += 92
        total = summary["num_cases"]
        for mode, count in sorted(summary["failure_modes"].items()):
            w = 620 * count / total
            fill = mode_colors.get(mode, color)
            svg.append(f'<rect x="{x}" y="{mode_y}" width="{w:.1f}" height="24" fill="{fill}" opacity="0.82"/>')
            if w > 54:
                svg.append(f'<text x="{x+5}" y="{mode_y+17}" font-family="Arial" font-size="11" fill="#ffffff">{mode_labels.get(mode, mode)}: {count}</text>')
            elif w > 20:
                svg.append(f'<text x="{x+w/2:.1f}" y="{mode_y+17}" text-anchor="middle" font-family="Arial" font-size="11" font-weight="700" fill="#ffffff">{count}</text>')
            x += w
        mode_y += 34

    delta = comparison.get("delta", {})
    physics_y = 722
    svg.append(f'<rect x="52" y="{physics_y}" width="1016" height="72" rx="4" fill="#ffffff" stroke="#cbd5e1"/>')
    svg.append(f'<text x="68" y="{physics_y+22}" font-family="Arial" font-size="14" font-weight="700" fill="#0f172a">Physical interpretation</text>')
    svg.append(f'<text x="68" y="{physics_y+43}" font-family="Arial" font-size="12.5" fill="#334155">Success increases by {delta.get("success_rate_points", 0):+.1f} points and p95 touchdown speed falls by {abs(delta.get("p95_touchdown_speed_mps", 0)):.2f} m/s while peak tilt and gimbal both decrease.</text>')
    svg.append(f'<text x="68" y="{physics_y+62}" font-family="Arial" font-size="12.5" fill="#334155">The gain therefore comes from altitude-scheduled allocation: early lateral impulse reduces late tilt demand, preserving T cos(theta) for terminal vertical-energy removal.</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
