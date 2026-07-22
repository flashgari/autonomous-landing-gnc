import unittest

from landing_gnc.sim import run_simulation


class SimulationTests(unittest.TestCase):
    def test_nominal_simulation_reaches_ground_with_propellant(self):
        rows, metrics, _ = run_simulation(duration_s=50.0, dt_s=0.1)
        self.assertGreater(len(rows), 10)
        self.assertLessEqual(rows[-1]["z_m"], 0.05)
        self.assertGreater(metrics["propellant_remaining_kg"], 0.0)
        self.assertTrue(metrics["success"])


if __name__ == "__main__":
    unittest.main()
