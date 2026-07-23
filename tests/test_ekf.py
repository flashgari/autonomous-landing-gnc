import math
import random
import unittest

import numpy as np

from landing_gnc.ekf import (
    AttitudeMeasurement,
    EkfSensorSuite,
    ErrorStateEkf,
    GpsMeasurement,
    ImuMeasurement,
)
from landing_gnc.models import EkfConfig, EkfSensorModel, Environment, FaultScenario, State
from landing_gnc.sim import run_simulation


class ErrorStateEkfTests(unittest.TestCase):
    def test_hover_accelerometer_reads_gravity_as_specific_force(self):
        model = EkfSensorModel(
            accel_noise_density_mps2_sqrt_hz=0.0,
            gyro_noise_density_radps_sqrt_hz=0.0,
            accel_initial_bias_sigma_mps2=0.0,
            gyro_initial_bias_sigma_radps=0.0,
            accel_bias_random_walk_mps2_sqrt_s=0.0,
            gyro_bias_random_walk_radps_sqrt_s=0.0,
        )
        environment = Environment()
        sensors = EkfSensorSuite(model, random.Random(3), environment)
        state = State(0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 24_000.0)
        measurement = sensors.measure_imu(state, 0.0, 0.0, 0.05)
        self.assertAlmostEqual(measurement.specific_force_body_x_mps2, 0.0, places=10)
        self.assertAlmostEqual(
            measurement.specific_force_body_z_mps2,
            environment.gravity_mps2,
            places=10,
        )

    def test_level_hover_propagation_preserves_nominal_state_and_covariance(self):
        environment = Environment()
        estimator = ErrorStateEkf(EkfConfig(), environment.gravity_mps2)
        gps = GpsMeasurement(0.0, 2.0, 100.0, 0.0, 0.0)
        attitude = AttitudeMeasurement(0.0, 0.0)
        imu0 = ImuMeasurement(0.0, 0.0, environment.gravity_mps2, 0.0)
        estimator.initialize(gps, attitude, imu0)
        estimator.propagate(
            ImuMeasurement(0.1, 0.0, environment.gravity_mps2, 0.0),
            0.1,
        )
        estimate = estimator.to_state(24_000.0)
        self.assertAlmostEqual(estimate.x_m, 2.0, places=9)
        self.assertAlmostEqual(estimate.z_m, 100.0, places=9)
        self.assertTrue(np.allclose(estimator.covariance, estimator.covariance.T))
        self.assertTrue(np.all(np.linalg.eigvalsh(estimator.covariance) > 0.0))

    def test_gps_dropout_remains_landable_and_radar_bias_is_gated(self):
        dropout = FaultScenario(
            name="gps_dropout",
            gps_dropout_start_s=8.0,
            gps_dropout_end_s=28.0,
        )
        _, dropout_metrics, _ = run_simulation(
            duration_s=70.0,
            dt_s=0.05,
            guidance_mode="corridor",
            navigation_mode="ekf",
            actuator_mode="flight_like",
            rng_seed=4242,
            fault=dropout,
        )
        self.assertTrue(dropout_metrics["success"])
        self.assertLess(abs(dropout_metrics["landing_x_error_m"]), 1.0)

        radar_bias = FaultScenario(
            name="radar_bias",
            altitude_bias_step_time_s=12.0,
            altitude_bias_step_m=12.0,
        )
        _, bias_metrics, _ = run_simulation(
            duration_s=70.0,
            dt_s=0.05,
            guidance_mode="corridor",
            navigation_mode="ekf",
            actuator_mode="flight_like",
            rng_seed=4242,
            fault=radar_bias,
        )
        self.assertTrue(bias_metrics["success"])
        self.assertGreater(bias_metrics["ekf_radar_rejected_updates"], 100)
        self.assertTrue(math.isfinite(bias_metrics["ekf_mean_nees"]))


if __name__ == "__main__":
    unittest.main()
