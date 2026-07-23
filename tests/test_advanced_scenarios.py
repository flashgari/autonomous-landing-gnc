import unittest

from landing_gnc.experiments import run_fault_and_divert_scenarios


class AdvancedScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.summary, _ = run_fault_and_divert_scenarios(dt_s=0.08)

    def test_sensor_fault_is_rejected_and_lands(self):
        metrics = self.summary["scenarios"]["altitude_sensor_fault"]["metrics"]
        self.assertTrue(metrics["success"])
        self.assertGreater(metrics["navigation_rejection_count"], 0)

    def test_hazard_divert_lands_clear(self):
        metrics = self.summary["scenarios"]["hazard_divert"]["metrics"]
        self.assertTrue(metrics["success"])
        self.assertGreater(metrics["hazard_clearance_m"], 3.0)

    def test_large_thrust_loss_exposes_terminal_boundary(self):
        metrics = self.summary["scenarios"]["thrust_loss"]["metrics"]
        self.assertFalse(metrics["success"])
        self.assertGreater(metrics["propellant_remaining_kg"], 1000.0)

    def test_smaller_thrust_loss_is_recoverable(self):
        metrics = self.summary["scenarios"]["thrust_loss_recoverable"]["metrics"]
        self.assertTrue(metrics["success"])


if __name__ == "__main__":
    unittest.main()
