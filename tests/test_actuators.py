import unittest

from landing_gnc.actuators import ActuatorStack
from landing_gnc.models import ActuatorModel, Command, Vehicle


class ActuatorTests(unittest.TestCase):
    def test_gimbal_and_throttle_rate_limits_are_respected(self):
        dt_s = 0.05
        model = ActuatorModel(command_delay_s=0.0)
        stack = ActuatorStack(model, Vehicle(), dt_s)
        command = Command(1.0, 0.2, 0.0, 0.0, 0.0)
        applied = stack.step(command, dt_s)
        self.assertLessEqual(applied.throttle, model.throttle_rate_limit_per_s * dt_s + 1e-12)
        self.assertLessEqual(applied.gimbal_rad, model.gimbal_rate_limit_radps * dt_s + 1e-12)


if __name__ == "__main__":
    unittest.main()
