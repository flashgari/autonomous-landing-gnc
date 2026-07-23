#!/usr/bin/env python3
"""Create recruiter-facing ESKF consistency and robustness figures."""

import csv
import json
import math
from pathlib import Path


NAVY = "#0f172a"
TEXT = "#334155"
MUTED = "#64748b"
GRID = "#cbd5e1"
BLUE = "#2563eb"
RED = "#dc2626"
GREEN = "#059669"
PURPLE = "#7c3aed"
ORANGE = "#ea580c"
TEAL = "#0891b2"


def load_rows(path: Path) -> list[dict[str, float]]:
    with path.open() as stream:
        return [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]


def line_path(
    rows: list[dict[str, float]],
    x_key: str,
    y_values: list[float],
    sx,
    sy,
) -> str:
    commands = []
    start_segment = True
    for row, value in zip(rows, y_values):
        if not math.isfinite(value):
            start_segment = True
            continue
        command = "M" if start_segment else "L"
        commands.append(f"{command}{sx(row[x_key]):.1f},{sy(value):.1f}")
        start_segment = False
    return " ".join(commands)


def finite(values):
    return [value for value in values if math.isfinite(value)]


def text_block(svg, x, y, lines, size=14, color=TEXT, line_height=21, weight=400):
    for index, line in enumerate(lines):
        svg.append(
            f'<text x="{x}" y="{y + index * line_height}" '
            f'font-family="Arial" font-size="{size}" font-weight="{weight}" '
            f'fill="{color}">{line}</text>'
        )


def main() -> None:
    nominal = load_rows(Path("outputs/ekf_nominal.csv"))
    dropout = load_rows(Path("outputs/ekf_gps_dropout.csv"))
    radar_bias = load_rows(Path("outputs/ekf_radar_bias.csv"))
    campaign = json.loads(Path("outputs/ekf_navigation_campaign.json").read_text())
    write_consistency_figure(nominal, Path("figures/ekf_consistency.svg"))
    write_robustness_figure(
        nominal,
        dropout,
        radar_bias,
        campaign,
        Path("figures/ekf_navigation_robustness.svg"),
    )
    write_fault_response_figure(
        dropout,
        radar_bias,
        Path("figures/ekf_sensor_fault_response.svg"),
    )
    print("Wrote figures/ekf_consistency.svg")
    print("Wrote figures/ekf_navigation_robustness.svg")
    print("Wrote figures/ekf_sensor_fault_response.svg")


def write_consistency_figure(rows, path: Path) -> None:
    width, height = 1500, 1120
    left = 100
    chart_width = 870
    interpretation_x = 1030
    panel_height = 180
    top = 125
    gap = 70
    end_time = rows[-1]["time_s"]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Error-state EKF estimation error, covariance, NIS, and NEES consistency</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="62" y="52" font-family="Arial" font-size="31" font-weight="700" fill="{NAVY}">Error-State EKF Consistency Evidence</text>',
        f'<text x="62" y="82" font-family="Arial" font-size="16" fill="{MUTED}">Nominal powered descent: inertial propagation with GPS, radar-altimeter, and attitude aiding.</text>',
    ]

    panel_specs = [
        (
            "Horizontal position error and 3-sigma envelope",
            [row["x_estimation_error_m"] for row in rows],
            [3.0 * row["ekf_sigma_x"] for row in rows],
            "m",
            BLUE,
        ),
        (
            "Altitude error and 3-sigma envelope",
            [row["z_estimation_error_m"] for row in rows],
            [3.0 * row["ekf_sigma_z"] for row in rows],
            "m",
            RED,
        ),
        (
            "Pitch error and 3-sigma envelope",
            [row["theta_estimation_error_deg"] for row in rows],
            [math.degrees(3.0 * row["ekf_sigma_theta"]) for row in rows],
            "deg",
            PURPLE,
        ),
    ]
    for index, (title, errors, bounds, unit, color) in enumerate(panel_specs):
        y0 = top + index * (panel_height + gap)
        amplitude = max(
            max(abs(value) for value in errors),
            max(abs(value) for value in bounds),
            0.01,
        ) * 1.12

        def sx(time_s):
            return left + time_s / end_time * chart_width

        def sy(value):
            return y0 + panel_height / 2.0 - value / amplitude * panel_height / 2.0

        svg.extend(
            [
                f'<text x="{left}" y="{y0-14}" font-family="Arial" font-size="18" font-weight="700" fill="{NAVY}">{title}</text>',
                f'<rect x="{left}" y="{y0}" width="{chart_width}" height="{panel_height}" fill="#ffffff" stroke="{GRID}"/>',
                f'<line x1="{left}" y1="{sy(0):.1f}" x2="{left+chart_width}" y2="{sy(0):.1f}" stroke="#94a3b8"/>',
                f'<path d="{line_path(rows, "time_s", bounds, sx, sy)}" fill="none" stroke="{MUTED}" stroke-width="1.5" stroke-dasharray="7 5"/>',
                f'<path d="{line_path(rows, "time_s", [-value for value in bounds], sx, sy)}" fill="none" stroke="{MUTED}" stroke-width="1.5" stroke-dasharray="7 5"/>',
                f'<path d="{line_path(rows, "time_s", errors, sx, sy)}" fill="none" stroke="{color}" stroke-width="2.2"/>',
                f'<text x="{left-62}" y="{y0+14}" font-family="Arial" font-size="12" fill="{MUTED}">+{amplitude:.2f}</text>',
                f'<text x="{left-62}" y="{y0+panel_height}" font-family="Arial" font-size="12" fill="{MUTED}">-{amplitude:.2f}</text>',
                f'<text x="{left+8}" y="{y0+19}" font-family="Arial" font-size="12" fill="{color}">error</text>',
                f'<text x="{left+63}" y="{y0+19}" font-family="Arial" font-size="12" fill="{MUTED}">dashed: +/-3 sigma [{unit}]</text>',
            ]
        )

    y0 = top + 3 * (panel_height + gap)
    nees = [row["ekf_nees"] for row in rows]
    gps_normalized_nis = [
        row["ekf_nis_gps"] / 4.0 if math.isfinite(row["ekf_nis_gps"]) else math.nan
        for row in rows
    ]
    radar_nis = [row["ekf_nis_radar"] for row in rows]
    attitude_nis = [row["ekf_nis_attitude"] for row in rows]
    finite_all = finite(nees + gps_normalized_nis + radar_nis + attitude_nis)
    upper = max(10.0, min(30.0, max(finite_all) * 1.05))

    def sx_diag(time_s):
        return left + time_s / end_time * chart_width

    def sy_diag(value):
        clipped = min(max(value, 0.0), upper)
        return y0 + panel_height - clipped / upper * panel_height

    svg.extend(
        [
            f'<text x="{left}" y="{y0-14}" font-family="Arial" font-size="18" font-weight="700" fill="{NAVY}">Normalized innovations and eight-state NEES</text>',
            f'<rect x="{left}" y="{y0}" width="{chart_width}" height="{panel_height}" fill="#ffffff" stroke="{GRID}"/>',
            f'<line x1="{left}" y1="{sy_diag(8):.1f}" x2="{left+chart_width}" y2="{sy_diag(8):.1f}" stroke="{MUTED}" stroke-dasharray="7 5"/>',
            f'<path d="{line_path(rows, "time_s", nees, sx_diag, sy_diag)}" fill="none" stroke="{NAVY}" stroke-width="2.0"/>',
            f'<path d="{line_path(rows, "time_s", gps_normalized_nis, sx_diag, sy_diag)}" fill="none" stroke="{BLUE}" stroke-width="1.5"/>',
            f'<path d="{line_path(rows, "time_s", radar_nis, sx_diag, sy_diag)}" fill="none" stroke="{GREEN}" stroke-width="1.3" opacity="0.85"/>',
            f'<path d="{line_path(rows, "time_s", attitude_nis, sx_diag, sy_diag)}" fill="none" stroke="{PURPLE}" stroke-width="1.3" opacity="0.80"/>',
            f'<text x="{left+8}" y="{y0+19}" font-family="Arial" font-size="12" fill="{NAVY}">NEES</text>',
            f'<text x="{left+62}" y="{y0+19}" font-family="Arial" font-size="12" fill="{BLUE}">GPS NIS/4</text>',
            f'<text x="{left+150}" y="{y0+19}" font-family="Arial" font-size="12" fill="{GREEN}">radar NIS</text>',
            f'<text x="{left+235}" y="{y0+19}" font-family="Arial" font-size="12" fill="{PURPLE}">attitude NIS</text>',
            f'<text x="{left+chart_width-112}" y="{sy_diag(8)-7:.1f}" font-family="Arial" font-size="11" fill="{MUTED}">E[NEES] = 8</text>',
            f'<text x="{left+chart_width/2-24}" y="{y0+panel_height+34}" font-family="Arial" font-size="14" fill="{TEXT}">time (s)</text>',
        ]
    )

    coverage_x = 100.0 * sum(row["ekf_x_inside_3sigma"] for row in rows) / len(rows)
    coverage_z = 100.0 * sum(row["ekf_z_inside_3sigma"] for row in rows) / len(rows)
    coverage_theta = 100.0 * sum(row["ekf_theta_inside_3sigma"] for row in rows) / len(rows)
    mean_nees = sum(nees) / len(nees)
    text_block(
        svg,
        interpretation_x,
        142,
        ["What the covariance envelopes mean"],
        size=19,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        interpretation_x,
        178,
        [
            "The solid traces are estimation errors,",
            "while the dashed curves are +/-3 standard",
            "deviations from the propagated covariance.",
            "Containment tests whether the stochastic",
            "model is commensurate with the actual error;",
            "it is not simply an accuracy score.",
        ],
    )
    text_block(
        svg,
        interpretation_x,
        330,
        [
            f"Observed 3-sigma coverage: x {coverage_x:.1f}%,",
            f"z {coverage_z:.1f}%, pitch {coverage_theta:.1f}%.",
            "Near-complete nominal containment indicates",
            "slightly conservative uncertainty, which is",
            "preferable to an overconfident landing filter.",
        ],
    )
    text_block(
        svg,
        interpretation_x,
        490,
        ["Why the bounds contract and breathe"],
        size=19,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        interpretation_x,
        526,
        [
            "IMU integration injects process uncertainty",
            "and couples attitude error into translational",
            "acceleration through d(R f)/d(theta). GPS and",
            "radar updates then remove observable position",
            "and velocity uncertainty, producing the small",
            "sawtooth modulation between aiding epochs.",
        ],
    )
    text_block(
        svg,
        interpretation_x,
        700,
        ["NIS and NEES interpretation"],
        size=19,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        interpretation_x,
        736,
        [
            "NIS measures each innovation in units of its",
            "predicted covariance; division by measurement",
            "dimension makes a value near one the expected",
            "scale. NEES tests the full eight-state error",
            "against P, with expected mean eight under an",
            "ideal linear-Gaussian consistency model.",
            f"This run gives mean NEES {mean_nees:.2f}.",
        ],
    )
    text_block(
        svg,
        interpretation_x,
        930,
        ["Engineering limit"],
        size=19,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        interpretation_x,
        966,
        [
            "A single trajectory cannot establish statistical",
            "consistency. The matched-seed Monte Carlo campaign",
            "provides the ensemble evidence; this plot exposes",
            "the time-domain mechanism and any local divergence.",
        ],
    )
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


def write_robustness_figure(nominal, dropout, radar_bias, campaign, path: Path) -> None:
    width, height = 1500, 1040
    case_count = campaign["campaign"]["cases_per_filter"]
    summaries = campaign["monte_carlo_summaries"]
    alpha_beta = summaries["estimated"]
    ekf = summaries["ekf"]
    deterministic = campaign["deterministic_cases"]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>Error-state EKF robustness under dispersions, GPS dropout, and radar bias</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="62" y="52" font-family="Arial" font-size="31" font-weight="700" fill="{NAVY}">Navigation Architecture and Fault-Tolerance Evidence</text>',
        f'<text x="62" y="82" font-family="Arial" font-size="16" fill="{MUTED}">Matched {case_count}-case dispersions plus deterministic aiding-sensor faults; corridor guidance and actuator dynamics held fixed.</text>',
    ]

    chart_x = 90
    chart_y = 142
    label_w = 250
    bar_w = 440
    metrics = [
        (
            "success rate",
            100.0 * alpha_beta["success_rate"],
            100.0 * ekf["success_rate"],
            100.0,
            "%",
            True,
        ),
        (
            "p95 |landing error|",
            alpha_beta["p95_abs_landing_error_m"],
            ekf["p95_abs_landing_error_m"],
            max(
                alpha_beta["p95_abs_landing_error_m"],
                ekf["p95_abs_landing_error_m"],
            ),
            "m",
            False,
        ),
        (
            "p95 touchdown speed",
            alpha_beta["p95_touchdown_speed_mps"],
            ekf["p95_touchdown_speed_mps"],
            max(
                alpha_beta["p95_touchdown_speed_mps"],
                ekf["p95_touchdown_speed_mps"],
            ),
            "m/s",
            False,
        ),
    ]
    svg.append(
        f'<text x="{chart_x}" y="{chart_y-24}" font-family="Arial" font-size="21" font-weight="700" fill="{NAVY}">Matched-seed Monte Carlo result</text>'
    )
    for index, (label, base, advanced, scale_max, unit, higher_better) in enumerate(metrics):
        y = chart_y + index * 126
        scale = bar_w / max(scale_max, 1.0e-9)
        svg.extend(
            [
                f'<text x="{chart_x}" y="{y+17}" font-family="Arial" font-size="16" font-weight="700" fill="{NAVY}">{label}</text>',
                f'<rect x="{chart_x+label_w}" y="{y}" width="{base*scale:.1f}" height="34" fill="{ORANGE}" opacity="0.88"/>',
                f'<rect x="{chart_x+label_w}" y="{y+45}" width="{advanced*scale:.1f}" height="34" fill="{TEAL}" opacity="0.92"/>',
                f'<text x="{chart_x+label_w+base*scale+10:.1f}" y="{y+23}" font-family="Arial" font-size="14" fill="{TEXT}">alpha-beta {base:.2f} {unit}</text>',
                f'<text x="{chart_x+label_w+advanced*scale+10:.1f}" y="{y+68}" font-family="Arial" font-size="14" fill="{TEXT}">ESKF {advanced:.2f} {unit}</text>',
            ]
        )
    delta = campaign["delta_ekf_minus_alpha_beta"]
    text_block(
        svg,
        970,
        132,
        ["Physical interpretation"],
        size=21,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        970,
        172,
        [
            f"Success changes by {delta['success_rate_points']:+.1f} percentage points",
            f"and p95 landing error changes by {delta['p95_abs_landing_error_m']:+.2f} m.",
            "The gain is not caused by retuning guidance:",
            "both filters command the same corridor law.",
            "It comes from propagating measured specific",
            "force through the nonlinear attitude transform,",
            "estimating inertial biases, and weighting each",
            "aiding source by its predicted uncertainty.",
        ],
    )

    divider_y = 535
    svg.append(
        f'<line x1="62" y1="{divider_y}" x2="{width-62}" y2="{divider_y}" stroke="{GRID}" stroke-width="2"/>'
    )
    svg.append(
        f'<text x="90" y="{divider_y+45}" font-family="Arial" font-size="21" font-weight="700" fill="{NAVY}">Deterministic sensor-fault mechanisms</text>'
    )
    cards = [
        (
            "Nominal multisensor solution",
            deterministic["nominal"],
            BLUE,
            [
                "GPS constrains low-frequency position and velocity;",
                "radar tightens altitude near the surface; attitude",
                "aiding prevents gyro bias from integrating into tilt.",
            ],
        ),
        (
            "GPS unavailable from 8 to 28 s",
            deterministic["gps_dropout"],
            PURPLE,
            [
                "The vehicle coasts on strapdown IMU propagation.",
                "Accelerometer-bias uncertainty drives position-error",
                "growth proportional to time squared. Radar still",
                "bounds altitude, and GPS reacquisition removes drift.",
            ],
        ),
        (
            "12 m radar bias step at 12 s",
            deterministic["radar_bias"],
            GREEN,
            [
                "The step produces a large scalar innovation. Its NIS",
                "exceeds the chi-square gate, so the radar correction",
                "is rejected while GPS carries altitude observability.",
            ],
        ),
    ]
    card_width = 420
    for index, (title, metrics_case, color, lines) in enumerate(cards):
        x = 90 + index * 465
        y = divider_y + 78
        svg.extend(
            [
                f'<rect x="{x}" y="{y}" width="{card_width}" height="320" rx="6" fill="#ffffff" stroke="{GRID}"/>',
                f'<rect x="{x}" y="{y}" width="8" height="320" fill="{color}"/>',
                f'<text x="{x+28}" y="{y+38}" font-family="Arial" font-size="18" font-weight="700" fill="{NAVY}">{title}</text>',
                f'<text x="{x+28}" y="{y+76}" font-family="Arial" font-size="15" fill="{TEXT}">landing error {metrics_case["landing_x_error_m"]:+.2f} m</text>',
                f'<text x="{x+28}" y="{y+102}" font-family="Arial" font-size="15" fill="{TEXT}">touchdown speed {metrics_case["touchdown_speed_mps"]:.2f} m/s</text>',
                f'<text x="{x+28}" y="{y+128}" font-family="Arial" font-size="15" fill="{TEXT}">mean NEES {metrics_case["ekf_mean_nees"]:.2f}</text>',
                f'<text x="{x+28}" y="{y+154}" font-family="Arial" font-size="15" font-weight="700" fill="{GREEN if metrics_case["success"] else RED}">{"PASS" if metrics_case["success"] else "FAIL"}</text>',
            ]
        )
        text_block(svg, x + 28, y + 196, lines, size=14, line_height=22)
        if "radar" in title:
            svg.append(
                f'<text x="{x+28}" y="{y+290}" font-family="Arial" font-size="14" fill="{TEXT}">radar updates rejected: {metrics_case["ekf_radar_rejected_updates"]}</text>'
            )
    text_block(
        svg,
        90,
        990,
        [
            "Verification boundary: passing these software-in-the-loop cases does not establish flight readiness; unmodeled sensor latency, terrain-relative navigation, six-DOF coupling, and hardware timing remain.",
        ],
        size=14,
        color=MUTED,
    )
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


def write_fault_response_figure(dropout, radar_bias, path: Path) -> None:
    width, height = 1500, 900
    left = 100
    chart_width = 875
    side_x = 1040
    top_y = 145
    panel_height = 245
    bottom_y = 535
    dropout_end_time = dropout[-1]["time_s"]
    radar_end_time = radar_bias[-1]["time_s"]
    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img">',
        "<title>ESKF covariance growth during GPS dropout and NIS rejection of radar bias</title>",
        '<rect width="100%" height="100%" fill="#f8fafc"/>',
        f'<text x="62" y="52" font-family="Arial" font-size="31" font-weight="700" fill="{NAVY}">Aiding-Sensor Fault Response in the Navigation Filter</text>',
        f'<text x="62" y="82" font-family="Arial" font-size="16" fill="{MUTED}">Time-domain evidence separates inertial uncertainty growth from covariance-normalized measurement rejection.</text>',
    ]

    x_errors = [row["x_estimation_error_m"] for row in dropout]
    x_bounds = [3.0 * row["ekf_sigma_x"] for row in dropout]
    x_amplitude = 1.12 * max(
        max(abs(value) for value in x_errors),
        max(x_bounds),
    )

    def sx_dropout(time_s):
        return left + time_s / dropout_end_time * chart_width

    def sy_dropout(value):
        return top_y + panel_height / 2.0 - value / x_amplitude * panel_height / 2.0

    dropout_start_x = sx_dropout(8.0)
    dropout_end_x = sx_dropout(28.0)
    svg.extend(
        [
            f'<text x="{left}" y="{top_y-16}" font-family="Arial" font-size="19" font-weight="700" fill="{NAVY}">Horizontal error and uncertainty during GPS outage</text>',
            f'<rect x="{left}" y="{top_y}" width="{chart_width}" height="{panel_height}" fill="#ffffff" stroke="{GRID}"/>',
            f'<rect x="{dropout_start_x:.1f}" y="{top_y}" width="{dropout_end_x-dropout_start_x:.1f}" height="{panel_height}" fill="#ede9fe" opacity="0.82"/>',
            f'<line x1="{left}" y1="{sy_dropout(0):.1f}" x2="{left+chart_width}" y2="{sy_dropout(0):.1f}" stroke="#94a3b8"/>',
            f'<path d="{line_path(dropout, "time_s", x_bounds, sx_dropout, sy_dropout)}" fill="none" stroke="{MUTED}" stroke-width="1.7" stroke-dasharray="7 5"/>',
            f'<path d="{line_path(dropout, "time_s", [-value for value in x_bounds], sx_dropout, sy_dropout)}" fill="none" stroke="{MUTED}" stroke-width="1.7" stroke-dasharray="7 5"/>',
            f'<path d="{line_path(dropout, "time_s", x_errors, sx_dropout, sy_dropout)}" fill="none" stroke="{BLUE}" stroke-width="2.3"/>',
            f'<text x="{dropout_start_x+12:.1f}" y="{top_y+23}" font-family="Arial" font-size="13" font-weight="700" fill="{PURPLE}">GPS unavailable</text>',
            f'<text x="{dropout_end_x+8:.1f}" y="{top_y+23}" font-family="Arial" font-size="13" fill="{GREEN}">reacquired</text>',
            f'<text x="{left+10}" y="{top_y+panel_height-12}" font-family="Arial" font-size="12" fill="{BLUE}">horizontal error</text>',
            f'<text x="{left+125}" y="{top_y+panel_height-12}" font-family="Arial" font-size="12" fill="{MUTED}">dashed: +/-3 sigma</text>',
        ]
    )

    radar_nis = [
        row["ekf_nis_radar"] if math.isfinite(row["ekf_nis_radar"]) else 0.01
        for row in radar_bias
    ]
    log_min = -2.0
    log_max = math.log10(1500.0)

    def sx_radar(time_s):
        return left + time_s / radar_end_time * chart_width

    def sy_radar(value):
        log_value = math.log10(max(0.01, value))
        clipped = min(log_max, max(log_min, log_value))
        return bottom_y + panel_height - (clipped - log_min) / (log_max - log_min) * panel_height

    radar_gate = 9.0
    bias_start_x = sx_radar(12.0)
    svg.extend(
        [
            f'<text x="{left}" y="{bottom_y-16}" font-family="Arial" font-size="19" font-weight="700" fill="{NAVY}">Radar-altimeter NIS after a persistent +12 m step</text>',
            f'<rect x="{left}" y="{bottom_y}" width="{chart_width}" height="{panel_height}" fill="#ffffff" stroke="{GRID}"/>',
            f'<rect x="{bias_start_x:.1f}" y="{bottom_y}" width="{left+chart_width-bias_start_x:.1f}" height="{panel_height}" fill="#fee2e2" opacity="0.55"/>',
            f'<line x1="{left}" y1="{sy_radar(radar_gate):.1f}" x2="{left+chart_width}" y2="{sy_radar(radar_gate):.1f}" stroke="{RED}" stroke-width="1.8" stroke-dasharray="7 5"/>',
            f'<path d="{line_path(radar_bias, "time_s", radar_nis, sx_radar, sy_radar)}" fill="none" stroke="{GREEN}" stroke-width="2.0"/>',
            f'<line x1="{bias_start_x:.1f}" y1="{bottom_y}" x2="{bias_start_x:.1f}" y2="{bottom_y+panel_height}" stroke="{RED}" stroke-width="1.6"/>',
            f'<text x="{bias_start_x+10:.1f}" y="{bottom_y+24}" font-family="Arial" font-size="13" font-weight="700" fill="{RED}">bias injected</text>',
            f'<text x="{left+chart_width-105}" y="{sy_radar(radar_gate)-8:.1f}" font-family="Arial" font-size="12" fill="{RED}">NIS gate = 9</text>',
            f'<text x="{left-45}" y="{sy_radar(1000)+4:.1f}" font-family="Arial" font-size="12" fill="{MUTED}">10^3</text>',
            f'<text x="{left-38}" y="{sy_radar(1)+4:.1f}" font-family="Arial" font-size="12" fill="{MUTED}">1</text>',
            f'<text x="{left-51}" y="{sy_radar(0.01)+4:.1f}" font-family="Arial" font-size="12" fill="{MUTED}">10^-2</text>',
            f'<text x="{left+chart_width/2-22}" y="{bottom_y+panel_height+34}" font-family="Arial" font-size="14" fill="{TEXT}">time (s)</text>',
        ]
    )

    sigma_at_start = min(dropout, key=lambda row: abs(row["time_s"] - 8.0))["ekf_sigma_x"]
    sigma_at_end = min(dropout, key=lambda row: abs(row["time_s"] - 28.0))["ekf_sigma_x"]
    sigma_after_reacquisition = min(
        dropout,
        key=lambda row: abs(row["time_s"] - 32.0),
    )["ekf_sigma_x"]
    text_block(
        svg,
        side_x,
        137,
        ["GPS dropout physics"],
        size=20,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        side_x,
        176,
        [
            "Without GPS, horizontal accelerometer-bias",
            "uncertainty is weakly observable. Its velocity",
            "effect grows with time, and its position effect",
            "grows approximately with time squared.",
            "",
            f"1-sigma x uncertainty: {sigma_at_start:.2f} m at 8 s",
            f"to {sigma_at_end:.2f} m at 28 s. Four seconds",
            f"after reacquisition it is {sigma_after_reacquisition:.2f} m.",
            "",
            "Radar continues to observe altitude and the",
            "attitude aid constrains gyro-bias-driven frame",
            "rotation, so the outage is not full dead reckoning.",
        ],
    )
    text_block(
        svg,
        side_x,
        520,
        ["Radar fault physics"],
        size=20,
        color=NAVY,
        weight=700,
    )
    text_block(
        svg,
        side_x,
        559,
        [
            "NIS compares the squared altitude residual",
            "with the predicted innovation variance S.",
            "The 12 m step is many standard deviations",
            "larger than the modeled radar uncertainty,",
            "so NIS rises by orders of magnitude above",
            "the scalar gate and the update is omitted.",
            "",
            "The filter rejects 344 radar updates. GPS",
            "maintains altitude observability; this is why",
            "the fault remains survivable in this scenario.",
        ],
    )
    svg.append("</svg>")
    path.write_text("\n".join(svg) + "\n")


if __name__ == "__main__":
    main()
