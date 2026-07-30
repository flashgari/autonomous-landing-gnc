"""Quaternion and vector utilities for the 6-DOF landing extension.

Quaternions are scalar-first and rotate body-frame vectors into the inertial
frame. Angular velocity is expressed in body coordinates.
"""

from __future__ import annotations

import math

import numpy as np


def normalize(vector: np.ndarray, fallback: np.ndarray | None = None) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm > 1.0e-12:
        return vector / norm
    if fallback is None:
        raise ValueError("cannot normalize a near-zero vector")
    return np.array(fallback, dtype=float)


def quaternion_normalize(quaternion: np.ndarray) -> np.ndarray:
    normalized = normalize(
        np.asarray(quaternion, dtype=float),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )
    return normalized if normalized[0] >= 0.0 else -normalized


def quaternion_conjugate(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=float)
    return np.array([w, -x, -y, -z])


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = np.asarray(left, dtype=float)
    rw, rx, ry, rz = np.asarray(right, dtype=float)
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ]
    )


def quaternion_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion_normalize(quaternion)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=float)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ]
        )
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (rotation[2, 1] - rotation[1, 2]) / scale,
                    0.25 * scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                ]
            )
        elif index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            quaternion = np.array(
                [
                    (rotation[0, 2] - rotation[2, 0]) / scale,
                    (rotation[0, 1] + rotation[1, 0]) / scale,
                    0.25 * scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                ]
            )
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            quaternion = np.array(
                [
                    (rotation[1, 0] - rotation[0, 1]) / scale,
                    (rotation[0, 2] + rotation[2, 0]) / scale,
                    (rotation[1, 2] + rotation[2, 1]) / scale,
                    0.25 * scale,
                ]
            )
    return quaternion_normalize(quaternion)


def quaternion_from_euler(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    cr, sr = math.cos(0.5 * roll_rad), math.sin(0.5 * roll_rad)
    cp, sp = math.cos(0.5 * pitch_rad), math.sin(0.5 * pitch_rad)
    cy, sy = math.cos(0.5 * yaw_rad), math.sin(0.5 * yaw_rad)
    return quaternion_normalize(
        np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ]
        )
    )


def quaternion_to_euler(quaternion: np.ndarray) -> np.ndarray:
    """Return 3-2-1 roll, pitch, and yaw angles in radians."""

    w, x, y, z = quaternion_normalize(quaternion)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_argument = 2.0 * (w * y - z * x)
    pitch = math.asin(float(np.clip(pitch_argument, -1.0, 1.0)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return np.array([roll, pitch, yaw])


def quaternion_error_vector(
    current_body_to_inertial: np.ndarray,
    desired_body_to_inertial: np.ndarray,
) -> np.ndarray:
    error = quaternion_multiply(
        quaternion_conjugate(current_body_to_inertial),
        desired_body_to_inertial,
    )
    if error[0] < 0.0:
        error = -error
    return 2.0 * error[1:]


def desired_attitude_from_thrust(
    thrust_inertial: np.ndarray,
    yaw_reference_rad: float = 0.0,
) -> np.ndarray:
    body_z_inertial = normalize(
        np.asarray(thrust_inertial, dtype=float),
        np.array([0.0, 0.0, 1.0]),
    )
    horizontal_reference = np.array(
        [math.cos(yaw_reference_rad), math.sin(yaw_reference_rad), 0.0]
    )
    body_x_inertial = (
        horizontal_reference
        - np.dot(horizontal_reference, body_z_inertial)
        * body_z_inertial
    )
    if np.linalg.norm(body_x_inertial) < 1.0e-8:
        horizontal_reference = np.array(
            [-math.sin(yaw_reference_rad), math.cos(yaw_reference_rad), 0.0]
        )
        body_x_inertial = (
            horizontal_reference
            - np.dot(horizontal_reference, body_z_inertial)
            * body_z_inertial
        )
    body_x_inertial = normalize(body_x_inertial)
    body_y_inertial = normalize(np.cross(body_z_inertial, body_x_inertial))
    rotation = np.column_stack((body_x_inertial, body_y_inertial, body_z_inertial))
    return matrix_to_quaternion(rotation)


def tilt_from_vertical_rad(quaternion: np.ndarray) -> float:
    body_z_inertial = quaternion_to_matrix(quaternion)[:, 2]
    return math.acos(float(np.clip(body_z_inertial[2], -1.0, 1.0)))
