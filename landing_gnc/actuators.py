"""Throttle and TVC actuator command-path dynamics."""

from collections import deque
from dataclasses import dataclass

from .dynamics import clamp
from .models import ActuatorModel, Command, Vehicle


@dataclass
class ActuatorState:
    throttle: float = 0.0
    gimbal_rad: float = 0.0


class ActuatorStack:
    def __init__(self, model: ActuatorModel, vehicle: Vehicle, dt_s: float) -> None:
        self.model = model
        self.vehicle = vehicle
        self.state = ActuatorState()
        delay_steps = max(0, round(model.command_delay_s / dt_s))
        zero = Command(0.0, 0.0, 0.0, 0.0, 0.0)
        self.buffer = deque([zero] * (delay_steps + 1), maxlen=delay_steps + 1)

    def step(self, commanded: Command, dt_s: float) -> Command:
        self.buffer.append(commanded)
        delayed = self.buffer[0]
        throttle_target = apply_deadband(delayed.throttle, self.state.throttle, self.model.throttle_deadband)
        gimbal_target = apply_deadband(delayed.gimbal_rad, self.state.gimbal_rad, self.model.gimbal_deadband_rad)

        throttle_lagged = first_order_step(
            self.state.throttle,
            throttle_target,
            self.model.throttle_time_constant_s,
            dt_s,
        )
        gimbal_lagged = first_order_step(
            self.state.gimbal_rad,
            gimbal_target,
            self.model.gimbal_time_constant_s,
            dt_s,
        )
        self.state.throttle = rate_limit(
            self.state.throttle,
            throttle_lagged,
            self.model.throttle_rate_limit_per_s,
            dt_s,
        )
        self.state.gimbal_rad = rate_limit(
            self.state.gimbal_rad,
            gimbal_lagged,
            self.model.gimbal_rate_limit_radps,
            dt_s,
        )
        self.state.throttle = clamp(self.state.throttle, 0.0, 1.0)
        self.state.gimbal_rad = clamp(
            self.state.gimbal_rad,
            -self.vehicle.max_gimbal_rad,
            self.vehicle.max_gimbal_rad,
        )
        return Command(
            throttle=self.state.throttle,
            gimbal_rad=self.state.gimbal_rad,
            desired_theta_rad=commanded.desired_theta_rad,
            desired_ax_mps2=commanded.desired_ax_mps2,
            desired_az_mps2=commanded.desired_az_mps2,
        )


def first_order_step(current: float, target: float, time_constant_s: float, dt_s: float) -> float:
    if time_constant_s <= 0.0:
        return target
    return current + dt_s / (time_constant_s + dt_s) * (target - current)


def rate_limit(current: float, target: float, rate_per_s: float, dt_s: float) -> float:
    max_change = max(0.0, rate_per_s) * dt_s
    return current + clamp(target - current, -max_change, max_change)


def apply_deadband(target: float, current: float, deadband: float) -> float:
    return current if abs(target - current) < deadband else target
