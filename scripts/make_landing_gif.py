#!/usr/bin/env python3
"""Generate a GitHub-renderable GIF preview of the hazard-divert landing."""

import csv
import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT = 1200, 680
COLORS = {
    "background": "#f8fafc",
    "foreground": "#0f172a",
    "muted": "#475569",
    "grid": "#cbd5e1",
    "truth": "#2563eb",
    "estimate": "#7c3aed",
    "safe": "#059669",
    "danger": "#dc2626",
    "thrust": "#ea580c",
}


def font(size, bold=False):
    candidates = [
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold else "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Helvetica.ttc",
    ]
    for candidate in candidates:
        try:
            return ImageFont.truetype(candidate, size=size)
        except OSError:
            continue
    return ImageFont.load_default()


def load_rows(path):
    with path.open() as f:
        raw = [{key: float(value) for key, value in row.items()} for row in csv.DictReader(f)]
    stride = max(1, len(raw) // 105)
    rows = raw[::stride]
    if rows[-1] != raw[-1]:
        rows.append(raw[-1])
    return rows


def rotate(point, angle_rad):
    x, y = point
    return (
        x * math.cos(angle_rad) - y * math.sin(angle_rad),
        x * math.sin(angle_rad) + y * math.cos(angle_rad),
    )


def draw_wrapped(draw, text, xy, max_width, font_obj, fill, line_gap=4):
    words = text.split()
    lines = []
    current = ""
    for word in words:
        candidate = f"{current} {word}".strip()
        if current and draw.textbbox((0, 0), candidate, font=font_obj)[2] > max_width:
            lines.append(current)
            current = word
        else:
            current = candidate
    if current:
        lines.append(current)
    x, y = xy
    line_height = draw.textbbox((0, 0), "Ag", font=font_obj)[3]
    for line in lines:
        draw.text((x, y), line, fill=fill, font=font_obj)
        y += line_height + line_gap
    return y


def phase_description(row, z_max):
    altitude_ratio = row["z_m"] / max(z_max, 1.0)
    if altitude_ratio > 0.68:
        return (
            "Divert acquisition",
            "The guidance law tilts the thrust vector early, while time-to-go is large, to build the lateral impulse required by the new target.",
        )
    if altitude_ratio > 0.25:
        return (
            "Crossrange braking",
            "The curvature reverses as the controller changes lateral acceleration sign. This arrests horizontal velocity before the terminal corridor tightens.",
        )
    if altitude_ratio > 0.06:
        return (
            "Terminal alignment",
            "Tilt demand is reduced so the vertical projection T cos(theta) can remove descent energy. Remaining crossrange error is corrected within the landing corridor.",
        )
    return (
        "Touchdown shaping",
        "The controller drives lateral velocity toward zero and regulates vertical speed while finite throttle and gimbal dynamics limit the final correction rate.",
    )


def main():
    rows = load_rows(Path("outputs/scenario_hazard_divert.csv"))
    summary = json.loads(Path("outputs/advanced_scenarios.json").read_text())
    metrics = summary["scenarios"]["hazard_divert"]["metrics"]
    x_values = [value for row in rows for value in (row["x_m"], row["estimated_x_m"])]
    x_min, x_max = min(x_values + [-4.0, rows[0]["target_x_m"]]) - 4.0, max(x_values + [4.0, rows[0]["target_x_m"]]) + 4.0
    z_max = max(row["z_m"] for row in rows)

    def sx(x_m):
        return 78 + (x_m - x_min) / (x_max - x_min) * 680

    def sy(z_m):
        return 565 - z_m / z_max * 420

    title_font = font(30, bold=True)
    section_font = font(18, bold=True)
    label_font = font(15, bold=True)
    small_font = font(13)
    body_font = font(14)
    frames = []
    for index, row in enumerate(rows):
        image = Image.new("RGB", (WIDTH, HEIGHT), COLORS["background"])
        draw = ImageDraw.Draw(image)
        draw.text((40, 24), "Hazard-Relative Autonomous Landing", fill=COLORS["foreground"], font=title_font)
        draw.text((41, 63), "Guidance, navigation, rigid-body motion, and finite-rate actuator response", fill=COLORS["muted"], font=small_font)

        legend_y = 100
        draw.line((78, legend_y, 110, legend_y), fill=COLORS["truth"], width=4)
        draw.text((118, legend_y - 8), "true trajectory", fill=COLORS["muted"], font=small_font)
        draw.line((245, legend_y, 277, legend_y), fill=COLORS["estimate"], width=2)
        draw.text((285, legend_y - 8), "navigation estimate", fill=COLORS["muted"], font=small_font)
        draw.line((455, legend_y, 487, legend_y), fill=COLORS["thrust"], width=4)
        draw.text((495, legend_y - 8), "applied thrust vector", fill=COLORS["muted"], font=small_font)

        draw.line((78, 565, 758, 565), fill=COLORS["grid"], width=2)
        for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
            z_tick = fraction * z_max
            y_tick = sy(z_tick)
            draw.line((78, y_tick, 758, y_tick), fill="#e2e8f0", width=1)
            draw.text((35, y_tick - 7), f"{z_tick:.0f}", fill=COLORS["muted"], font=small_font)
        draw.text((34, 124), "z (m)", fill=COLORS["muted"], font=small_font)
        for x_tick in range(math.ceil(x_min / 5) * 5, math.floor(x_max / 5) * 5 + 1, 5):
            x_tick_px = sx(x_tick)
            draw.line((x_tick_px, 145, x_tick_px, 565), fill="#f1f5f9", width=1)
            draw.text((x_tick_px - 9, 575), f"{x_tick}", fill=COLORS["muted"], font=small_font)
        draw.text((78, 606), "downrange x (m)", fill=COLORS["muted"], font=small_font)

        hazard_left, hazard_right = sx(-4.0), sx(4.0)
        draw.rectangle((hazard_left, 544, hazard_right, 565), fill=COLORS["danger"])
        draw.text((hazard_left, 522), "debris hazard [-4, 4] m", fill=COLORS["danger"], font=small_font)
        target_x = sx(row["target_x_m"])
        draw.rectangle((target_x - 24, 558, target_x + 24, 570), fill=COLORS["safe"])
        draw.text((target_x - 44, 624), "target = 12 m", fill=COLORS["safe"], font=small_font)

        true_points = [(sx(item["x_m"]), sy(item["z_m"])) for item in rows[: index + 1]]
        estimate_points = [(sx(item["estimated_x_m"]), sy(item["estimated_z_m"])) for item in rows[: index + 1]]
        if len(true_points) > 1:
            draw.line(true_points, fill=COLORS["truth"], width=4)
            for start in range(0, len(estimate_points) - 1, 4):
                draw.line(estimate_points[start : start + 3], fill=COLORS["estimate"], width=2)

        px, py = sx(row["x_m"]), sy(row["z_m"])
        theta = math.radians(row["theta_deg"])
        body = [(-9, 20), (-9, -20), (0, -31), (9, -20), (9, 20)]
        body_points = [(px + rotate(point, theta)[0], py + rotate(point, theta)[1]) for point in body]
        draw.polygon(body_points, fill=COLORS["truth"], outline=COLORS["foreground"])
        thrust_length = 16 + 30 * row["throttle"]
        thrust_tip = rotate((0, thrust_length), theta + math.radians(row["gimbal_deg"]))
        draw.line((px, py + 20, px + thrust_tip[0], py + 20 + thrust_tip[1]), fill=COLORS["thrust"], width=4)
        draw.ellipse((sx(row["estimated_x_m"]) - 5, sy(row["estimated_z_m"]) - 5, sx(row["estimated_x_m"]) + 5, sy(row["estimated_z_m"]) + 5), fill=COLORS["estimate"])

        speed = math.hypot(row["vx_mps"], row["vz_mps"])
        nav_separation = math.hypot(
            row["estimated_x_m"] - row["x_m"],
            row["estimated_z_m"] - row["z_m"],
        )
        phase, explanation = phase_description(row, z_max)
        panel_x = 800
        draw.rounded_rectangle((panel_x, 98, 1164, 640), radius=6, fill="#ffffff", outline=COLORS["grid"], width=2)
        draw.text((panel_x + 20, 120), "Current flight phase", fill=COLORS["muted"], font=small_font)
        draw.text((panel_x + 20, 143), phase, fill=COLORS["foreground"], font=section_font)
        y_text = draw_wrapped(
            draw,
            explanation,
            (panel_x + 20, 176),
            324,
            body_font,
            COLORS["muted"],
            line_gap=4,
        )
        y_text += 13
        draw.text((panel_x + 20, y_text), "Coupled dynamics", fill=COLORS["foreground"], font=label_font)
        y_text += 25
        draw.text((panel_x + 20, y_text), "a_x = T sin(theta) / m", fill=COLORS["muted"], font=body_font)
        draw.text((panel_x + 20, y_text + 22), "a_z = T cos(theta) / m - g", fill=COLORS["muted"], font=body_font)
        draw.text((panel_x + 20, y_text + 44), "I theta_ddot = T L sin(delta)", fill=COLORS["muted"], font=body_font)
        y_text += 80
        draw.text((panel_x + 20, y_text), "State at this frame", fill=COLORS["foreground"], font=label_font)
        stats = [
            ("time", f'{row["time_s"]:.1f} s'),
            ("altitude", f'{row["z_m"]:.1f} m'),
            ("speed", f"{speed:.2f} m/s"),
            ("target error", f'{row["target_error_m"]:.2f} m'),
            ("throttle", f'{100*row["throttle"]:.0f}%'),
            ("nav separation", f"{nav_separation:.2f} m"),
        ]
        for stat_index, (label, value) in enumerate(stats):
            stat_x = panel_x + 20 + (stat_index % 2) * 166
            stat_y = y_text + 27 + (stat_index // 2) * 49
            draw.text((stat_x, stat_y), label, fill=COLORS["muted"], font=small_font)
            draw.text((stat_x, stat_y + 17), value, fill=COLORS["foreground"], font=label_font)
        draw.text((panel_x + 20, 552), "Estimator note", fill=COLORS["foreground"], font=label_font)
        draw_wrapped(
            draw,
            "Purple jumps are discrete estimator corrections from noisy measurements; they are not physical vehicle zigzags.",
            (panel_x + 20, 576),
            324,
            small_font,
            COLORS["muted"],
            line_gap=3,
        )
        draw.text(
            (78, 652),
            f"Outcome: landing x = {metrics['landing_x_m']:.2f} m, target error = {metrics['landing_x_error_m']:.2f} m, hazard clearance = {metrics['hazard_clearance_m']:.2f} m",
            fill=COLORS["foreground"],
            font=label_font,
        )
        frames.append(image)

    output = Path("media/hazard_divert_landing_preview.gif")
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=65,
        loop=0,
        optimize=True,
        disposal=1,
    )
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
