#!/usr/bin/env python3
"""Generate a GitHub-renderable preview of the 3D 6-DOF landing."""

import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


WIDTH = 1040
HEIGHT = 600
BACKGROUND = "#f8fafc"
PANEL = "#ffffff"
INK = "#0f172a"
MUTED = "#475569"
BORDER = "#cbd5e1"
BLUE = "#2563eb"
GREEN = "#059669"
ORANGE = "#ea580c"
RED = "#dc2626"
VIOLET = "#7c3aed"


def font(size, bold=False):
    candidates = [
        (
            "/System/Library/Fonts/Supplemental/Arial Bold.ttf"
            if bold
            else "/System/Library/Fonts/Supplemental/Arial.ttf"
        ),
        (
            "/System/Library/Fonts/Supplemental/Helvetica.ttc"
        ),
    ]
    for candidate in candidates:
        if Path(candidate).exists():
            return ImageFont.truetype(candidate, size)
    return ImageFont.load_default()


def load_rows():
    with Path("outputs/sixdof_crosswind.csv").open() as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def project(x_value, y_value, z_value):
    return (
        480 + 8.0 * x_value - 5.0 * y_value,
        500 - 0.56 * z_value + 1.15 * x_value + 0.70 * y_value,
    )


def rotation_matrix(row):
    w, x, y, z = (
        row["qw"],
        row["qx"],
        row["qy"],
        row["qz"],
    )
    return (
        (
            1 - 2 * (y * y + z * z),
            2 * (x * y - w * z),
            2 * (x * z + w * y),
        ),
        (
            2 * (x * y + w * z),
            1 - 2 * (x * x + z * z),
            2 * (y * z - w * x),
        ),
        (
            2 * (x * z - w * y),
            2 * (y * z + w * x),
            1 - 2 * (x * x + y * y),
        ),
    )


def phase(row):
    altitude = row["z_m"]
    if altitude > 90:
        return (
            "BRAKING",
            "Remove vertical kinetic energy while time-to-go supports lateral correction.",
        )
    if altitude > 25:
        return (
            "APPROACH",
            "Contract the horizontal corridor before tilt competes with vertical authority.",
        )
    if altitude > 6:
        return "FLARE", "Reduce descent kinetic energy and arrest residual horizontal motion."
    return "TERMINAL", "Hold the thrust axis near vertical at the 1.2 m/s descent reference."


def draw_frame(rows, index):
    image = Image.new("RGB", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(image)
    draw.text((34, 24), "3D 6-DOF Crosswind Landing", font=font(30, True), fill=INK)
    draw.text(
        (34, 62),
        "quaternion attitude, mass-scheduled inertia, and four-engine TVC allocation",
        font=font(15),
        fill=MUTED,
    )
    draw.rounded_rectangle(
        (28, 94, 1012, 526),
        radius=5,
        fill=PANEL,
        outline=BORDER,
        width=1,
    )

    for value in range(-20, 46, 5):
        draw.line(
            (project(value, -20, 0), project(value, 45, 0)),
            fill=BORDER,
            width=1,
        )
        draw.line(
            (project(-20, value, 0), project(45, value, 0)),
            fill=BORDER,
            width=1,
        )
    target = project(0, 0, 0)
    draw.ellipse(
        (
            target[0] - 10,
            target[1] - 10,
            target[0] + 10,
            target[1] + 10,
        ),
        outline=INK,
        width=2,
    )
    draw.text((target[0] + 14, target[1] - 8), "target", font=font(12, True), fill=INK)

    full_path = [
        project(row["x_m"], row["y_m"], row["z_m"])
        for row in rows
    ]
    traversed = full_path[: index + 1]
    draw.line(full_path, fill=BORDER, width=3, joint="curve")
    if len(traversed) > 1:
        draw.line(traversed, fill=BLUE, width=5, joint="curve")

    row = rows[index]
    origin = project(row["x_m"], row["y_m"], row["z_m"])
    rotation = rotation_matrix(row)
    body_z = (
        rotation[0][2],
        rotation[1][2],
        rotation[2][2],
    )
    tail = project(
        row["x_m"] - 6.0 * body_z[0],
        row["y_m"] - 6.0 * body_z[1],
        row["z_m"] - 6.0 * body_z[2],
    )
    nose = project(
        row["x_m"] + 7.0 * body_z[0],
        row["y_m"] + 7.0 * body_z[1],
        row["z_m"] + 7.0 * body_z[2],
    )
    draw.line((tail, nose), fill=INK, width=11)
    draw.ellipse(
        (nose[0] - 6, nose[1] - 6, nose[0] + 6, nose[1] + 6),
        fill=BLUE,
    )
    maximum_throttle = max(
        row[f"engine_{engine}_throttle"]
        for engine in range(1, 5)
    )
    plume = project(
        row["x_m"] - (6.0 + 11.0 * maximum_throttle) * body_z[0],
        row["y_m"] - (6.0 + 11.0 * maximum_throttle) * body_z[1],
        row["z_m"] - (6.0 + 11.0 * maximum_throttle) * body_z[2],
    )
    draw.line((tail, plume), fill=ORANGE, width=5)

    axis_colors = (RED, GREEN, BLUE)
    for column, color in enumerate(axis_colors):
        vector = (
            rotation[0][column] * 5.0,
            rotation[1][column] * 5.0,
            rotation[2][column] * 5.0,
        )
        tip = project(
            row["x_m"] + vector[0],
            row["y_m"] + vector[1],
            row["z_m"] + vector[2],
        )
        draw.line((origin, tip), fill=color, width=3)

    wind_start = project(-12, 30, 0)
    wind_end = project(-12 + 12 * 1.5, 30 - 6 * 1.5, 0)
    draw.line((wind_start, wind_end), fill=VIOLET, width=4)
    draw.text(
        (wind_start[0] - 10, wind_start[1] - 24),
        "13.4 m/s wind",
        font=font(12, True),
        fill=VIOLET,
    )

    stat_x = 765
    draw.text((stat_x, 120), f"t = {row['time_s']:.1f} s", font=font(19, True), fill=INK)
    stats = [
        ("altitude", f"{row['z_m']:.1f} m"),
        ("target error", f"{row['horizontal_error_m']:.2f} m"),
        ("vertical speed", f"{row['vz_mps']:.2f} m/s"),
        ("body tilt", f"{row['tilt_deg']:.2f} deg"),
        ("propellant", f"{row['propellant_remaining_kg']:.0f} kg"),
    ]
    y_value = 158
    for label, value in stats:
        draw.text((stat_x, y_value), label.upper(), font=font(10, True), fill=MUTED)
        draw.text((stat_x, y_value + 15), value, font=font(18, True), fill=INK)
        y_value += 53

    draw.text((stat_x, 432), "ENGINE THROTTLE", font=font(10, True), fill=MUTED)
    for engine in range(1, 5):
        throttle = row[f"engine_{engine}_throttle"]
        y_bar = 453 + 16 * (engine - 1)
        draw.text((stat_x, y_bar - 2), f"E{engine}", font=font(10, True), fill=INK)
        draw.rectangle((stat_x + 24, y_bar, stat_x + 184, y_bar + 8), fill=BORDER)
        draw.rectangle(
            (
                stat_x + 24,
                y_bar,
                stat_x + 24 + 160 * throttle,
                y_bar + 8,
            ),
            fill=VIOLET,
        )

    phase_name, explanation = phase(row)
    draw.rectangle((28, 540, 1012, 590), fill=PANEL, outline=BORDER)
    draw.text((44, 552), phase_name, font=font(13, True), fill=GREEN)
    draw.text((156, 551), explanation, font=font(13), fill=MUTED)
    draw.text(
        (850, 575),
        f"|q|-1 = {abs(row['quaternion_norm']-1):.1e}",
        font=font(10, True),
        fill=MUTED,
    )
    return image


def main():
    rows = load_rows()
    frame_count = 76
    indices = [
        round(index * (len(rows) - 1) / (frame_count - 1))
        for index in range(frame_count)
    ]
    frames = [draw_frame(rows, index) for index in indices]
    output = Path("media/sixdof_landing_preview.gif")
    output.parent.mkdir(parents=True, exist_ok=True)
    frames[0].save(
        output,
        save_all=True,
        append_images=frames[1:],
        duration=85,
        loop=0,
        optimize=True,
    )
    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
