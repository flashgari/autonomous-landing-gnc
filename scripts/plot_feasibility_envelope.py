#!/usr/bin/env python3
"""Plot landing feasibility and propellant margin over initial conditions."""

import csv
import json
from pathlib import Path


def main():
    with Path("outputs/landing_feasibility_envelope.csv").open() as f:
        rows = list(csv.DictReader(f))
    summary = json.loads(Path("outputs/landing_feasibility_summary.json").read_text())
    write_svg(rows, summary, Path("figures/landing_feasibility_envelope.svg"))
    print("Wrote figures/landing_feasibility_envelope.svg")


def write_svg(rows, summary, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    width, height = 1000, 650
    altitudes = sorted({float(row["initial_altitude_m"]) for row in rows}, reverse=True)
    offsets = sorted({float(row["initial_offset_m"]) for row in rows})
    lookup = {(float(row["initial_altitude_m"]), float(row["initial_offset_m"])): row for row in rows}
    grid_x, grid_y = 170, 110
    cell_w, cell_h = 118, 78
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Powered landing feasibility envelope</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        '<text x="52" y="42" font-family="Arial" font-size="24" font-weight="700" fill="#0f172a">Landing Feasibility and Propellant Margin</text>',
        '<text x="52" y="67" font-family="Arial" font-size="14" fill="#475569">Truth-state corridor guidance with flight-like throttle and gimbal dynamics.</text>',
    ]
    for col, offset in enumerate(offsets):
        x = grid_x + col * cell_w
        svg.append(f'<text x="{x+cell_w/2}" y="{grid_y-18}" text-anchor="middle" font-family="Arial" font-size="13" fill="#334155">{offset:.0f} m</text>')
    for row_index, altitude in enumerate(altitudes):
        y = grid_y + row_index * cell_h
        svg.append(f'<text x="{grid_x-18}" y="{y+cell_h/2+4}" text-anchor="end" font-family="Arial" font-size="13" fill="#334155">{altitude:.0f} m</text>')
        for col, offset in enumerate(offsets):
            x = grid_x + col * cell_w
            item = lookup[(altitude, offset)]
            success = item["success"] == "True"
            fill = "#dcfce7" if success else "#fee2e2"
            stroke = "#16a34a" if success else "#dc2626"
            label = f'{float(item["propellant_remaining_kg"]):.0f} kg' if success else "constraint fail"
            detail = f'err {float(item["landing_x_error_m"]):.1f} m'
            svg.append(f'<rect x="{x}" y="{y}" width="{cell_w-8}" height="{cell_h-8}" fill="{fill}" stroke="{stroke}"/>')
            svg.append(f'<text x="{x+(cell_w-8)/2}" y="{y+31}" text-anchor="middle" font-family="Arial" font-size="13" font-weight="700" fill="#0f172a">{label}</text>')
            svg.append(f'<text x="{x+(cell_w-8)/2}" y="{y+51}" text-anchor="middle" font-family="Arial" font-size="11" fill="#475569">{detail}</text>')
    svg.append(f'<text x="{grid_x+len(offsets)*cell_w/2-62}" y="{grid_y+len(altitudes)*cell_h+35}" font-family="Arial" font-size="14" fill="#334155">initial lateral offset</text>')
    svg.append(f'<text x="44" y="{grid_y+len(altitudes)*cell_h/2+55}" font-family="Arial" font-size="14" fill="#334155" transform="rotate(-90 44,{grid_y+len(altitudes)*cell_h/2+55})">initial altitude</text>')
    svg.append(f'<text x="52" y="{height-58}" font-family="Arial" font-size="14" font-weight="700" fill="#0f172a">Envelope result</text>')
    svg.append(f'<text x="52" y="{height-34}" font-family="Arial" font-size="13" fill="#475569">{summary["successes"]}/{summary["num_cases"]} grid points satisfy pad, velocity, tilt, and propellant constraints; green labels report residual propellant, not a binary score.</text>')
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
