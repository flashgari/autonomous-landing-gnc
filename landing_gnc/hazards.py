"""Hazard-zone geometry and deterministic landing-target selection."""

from .models import HazardZone


def point_clearance_m(x_m: float, hazards: tuple[HazardZone, ...]) -> float:
    if not hazards:
        return float("inf")
    clearances = []
    for zone in hazards:
        if zone.left_m <= x_m <= zone.right_m:
            return -min(x_m - zone.left_m, zone.right_m - x_m)
        clearances.append(min(abs(x_m - zone.left_m), abs(x_m - zone.right_m)))
    return min(clearances)


def select_safe_target(
    current_x_m: float,
    candidates_m: tuple[float, ...],
    hazards: tuple[HazardZone, ...],
    required_clearance_m: float = 3.0,
) -> float:
    safe = [x for x in candidates_m if point_clearance_m(x, hazards) >= required_clearance_m]
    if not safe:
        raise ValueError("no candidate landing target satisfies the hazard-clearance requirement")
    return min(safe, key=lambda x: (abs(x - current_x_m), abs(x)))
