import random
import unittest

from landing_gnc.models import EstimatorConfig, FaultScenario, SensorModel, State
from landing_gnc.navigation import AlphaBetaEstimator, SensorSuite


class NavigationTests(unittest.TestCase):
    def test_noise_free_constant_velocity_state_is_tracked(self):
        model = SensorModel(
            position_sigma_m=0.0,
            altitude_sigma_m=0.0,
            velocity_sigma_mps=0.0,
            attitude_sigma_rad=0.0,
            rate_sigma_radps=0.0,
            position_bias_sigma_m=0.0,
            altitude_bias_sigma_m=0.0,
            velocity_bias_sigma_mps=0.0,
            attitude_bias_sigma_rad=0.0,
            rate_bias_sigma_radps=0.0,
        )
        sensors = SensorSuite(model, random.Random(4))
        estimator = AlphaBetaEstimator(EstimatorConfig())
        for step in range(11):
            time_s = step * 0.1
            truth = State(time_s, 5.0 + 2.0 * time_s, 100.0 - 3.0 * time_s, 2.0, -3.0, 0.01, 0.0, 24_000.0)
            estimate = estimator.update(sensors.measure(truth), truth.mass_kg)
        self.assertAlmostEqual(estimate.x_m, truth.x_m, places=7)
        self.assertAlmostEqual(estimate.z_m, truth.z_m, places=7)
        self.assertAlmostEqual(estimate.vx_mps, truth.vx_mps, places=7)

    def test_large_altitude_bias_is_rejected(self):
        model = SensorModel(
            position_sigma_m=0.0,
            altitude_sigma_m=0.0,
            velocity_sigma_mps=0.0,
            attitude_sigma_rad=0.0,
            rate_sigma_radps=0.0,
            position_bias_sigma_m=0.0,
            altitude_bias_sigma_m=0.0,
            velocity_bias_sigma_mps=0.0,
            attitude_bias_sigma_rad=0.0,
            rate_bias_sigma_radps=0.0,
        )
        fault = FaultScenario(altitude_bias_step_time_s=0.1, altitude_bias_step_m=12.0)
        sensors = SensorSuite(model, random.Random(2), fault)
        estimator = AlphaBetaEstimator(EstimatorConfig())
        truth0 = State(0.0, 0.0, 100.0, 0.0, -2.0, 0.0, 0.0, 24_000.0)
        estimator.update(sensors.measure(truth0), truth0.mass_kg)
        truth1 = State(0.1, 0.0, 99.8, 0.0, -2.0, 0.0, 0.0, 24_000.0)
        estimate = estimator.update(sensors.measure(truth1), truth1.mass_kg)
        self.assertGreater(estimator.rejection_count, 0)
        self.assertLess(abs(estimate.z_m - truth1.z_m), 1.0)


if __name__ == "__main__":
    unittest.main()
