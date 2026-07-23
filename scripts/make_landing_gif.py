#!/usr/bin/env python3
"""Generate a GitHub-renderable GIF preview of the hazard-divert landing."""

import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


WIDTH, HEIGHT = 960, 540
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


def main():
    rows = load_rows(Path("outputs/scenario_hazard_divert.csv"))
    x_values = [value for row in rows for value in (row["x_m"], row["estimated_x_m"])]
    x_min, x_max = min(x_values + [-4.0, rows[0]["target_x_m"]]) - 4.0, max(x_values + [4.0, rows[0]["target_x_m"]]) + 4.0
    z_max = max(row["z_m"] for row in rows)

    def sx(x_m):
        return 72 + (x_m - x_min) / (x_max - x_min) * 816

    def sy(z_m):
        return 454 - z_m / z_max * 340

    title_font = font(28, bold=True)
    label_font = font(15, bold=True)
    small_font = font(13)
    frames = []
    for index, row in enumerate(rows):
        image = Image.new("RGB", (WIDTH, HEIGHT), COLORS["background"])
        draw = ImageDraw.Draw(image)
        draw.text((40, 24), "Hazard-Relative Autonomous Landing", fill=COLORS["foreground"], font=title_font)
        draw.text((41, 62), "true trajectory, navigation estimate, actuator response, and safe-target divert", fill=COLORS["muted"], font=small_font)

        draw.line((72, 454, 888, 454), fill=COLORS["grid"], width=2)
        hazard_left, hazard_right = sx(-4.0), sx(4.0)
        draw.rectangle((hazard_left, 435, hazard_right, 454), fill=COLORS["danger"])
        draw.text((hazard_left, 413), "debris hazard", fill=COLORS["danger"], font=small_font)
        target_x = sx(row["target_x_m"])
        draw.rectangle((target_x - 28, 448, target_x + 28, 460), fill=COLORS["safe"])
        draw.text((target_x - 46, 470), "safe target", fill=COLORS["safe"], font=small_font)

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
        stats = [
            ("time", f'{row["time_s"]:.1f} s'),
            ("altitude", f'{row["z_m"]:.1f} m'),
            ("target error", f'{row["target_error_m"]:.2f} m'),
            ("speed", f"{speed:.2f} m/s"),
            ("throttle", f'{100*row["throttle"]:.0f}%'),
        ]
        for stat_index, (label, value) in enumerate(stats):
            x = 42 + stat_index * 180
            draw.text((x, 500), label, fill=COLORS["muted"], font=small_font)
            draw.text((x, 517), value, fill=COLORS["foreground"], font=label_font)
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
