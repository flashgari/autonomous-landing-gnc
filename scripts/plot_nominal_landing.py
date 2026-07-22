#!/usr/bin/env python3
"""Generate SVG plots for the nominal landing run using only the standard library."""

import csv
from pathlib import Path


def load_rows(path):
    with path.open() as f:
        return [{k: float(v) if k != "success" else v for k, v in row.items()} for row in csv.DictReader(f)]


def points(rows, x_key, y_key):
    return [(r[x_key], r[y_key]) for r in rows]


def write_svg(rows, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    series = [
        ("Altitude", "z_m", "m", "#2563eb"),
        ("Vertical velocity", "vz_mps", "m/s", "#dc2626"),
        ("Horizontal error", "x_m", "m", "#7c3aed"),
        ("Throttle", "throttle", "-", "#059669"),
        ("Gimbal", "gimbal_deg", "deg", "#ea580c"),
        ("Propellant remaining", "prop_remaining_kg", "kg", "#0891b2"),
    ]
    width = 1000
    panel_h = 122
    gap = 34
    margin_l = 86
    margin_r = 28
    top0 = 78
    height = top0 + len(series) * panel_h + (len(series) - 1) * gap + 60
    t0, t1 = rows[0]["time_s"], rows[-1]["time_s"]

    def sx(t):
        return margin_l + (t - t0) / (t1 - t0) * (width - margin_l - margin_r)

    def sy(value, lo, hi, top):
        if abs(hi - lo) < 1e-12:
            lo -= 1.0
            hi += 1.0
        return top + panel_h - (value - lo) / (hi - lo) * panel_h

    def path_from(data, lo, hi, top):
        parts = []
        for i, (t, y) in enumerate(data):
            parts.append(("M" if i == 0 else "L") + f"{sx(t):.1f},{sy(y, lo, hi, top):.1f}")
        return " ".join(parts)

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Nominal autonomous landing simulation plots</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="54" y="42" font-family="Arial" font-size="24" font-weight="700" fill="#0f172a">Nominal Powered Landing Simulation</text>',
        '<text x="54" y="64" font-family="Arial" font-size="14" fill="#475569">Guidance, throttle, gimbal, and mass-state histories from simulator CSV output.</text>',
    ]
    for i, (label, key, unit, color) in enumerate(series):
        top = top0 + i * (panel_h + gap)
        values = [r[key] for r in rows]
        lo, hi = min(values), max(values)
        pad = 0.08 * max(1.0, hi - lo)
        lo -= pad
        hi += pad
        svg.append(f'<rect x="{margin_l}" y="{top}" width="{width-margin_l-margin_r}" height="{panel_h}" fill="#ffffff" stroke="#cbd5e1"/>')
        svg.append(f'<text x="{margin_l}" y="{top-8}" font-family="Arial" font-size="16" font-weight="700" fill="#0f172a">{label}</text>')
        svg.append(f'<text x="24" y="{top+panel_h/2}" font-family="Arial" font-size="13" fill="#334155" transform="rotate(-90 24,{top+panel_h/2})">{unit}</text>')
        svg.append(f'<text x="{margin_l-68}" y="{top+14}" font-family="Arial" font-size="12" fill="#64748b">{hi:.1f}</text>')
        svg.append(f'<text x="{margin_l-68}" y="{top+panel_h}" font-family="Arial" font-size="12" fill="#64748b">{lo:.1f}</text>')
        svg.append(f'<path d="{path_from(points(rows, "time_s", key), lo, hi, top)}" fill="none" stroke="{color}" stroke-width="2.5"/>')
    svg.append(f'<text x="{width/2 - 28}" y="{height-22}" font-family="Arial" font-size="14" fill="#334155">time (s)</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


def main():
    rows = load_rows(Path("outputs/nominal_landing.csv"))
    write_svg(rows, Path("figures/nominal_landing_summary.svg"))
    print("Wrote figures/nominal_landing_summary.svg")


if __name__ == "__main__":
    main()

