import unittest

from landing_gnc.monte_carlo import run_monte_carlo


class MonteCarloTests(unittest.TestCase):
    def test_small_campaign_is_reproducible(self):
        rows_a, summary_a = run_monte_carlo(num_cases=8, seed=7, dt_s=0.12)
        rows_b, summary_b = run_monte_carlo(num_cases=8, seed=7, dt_s=0.12)
        self.assertEqual(rows_a, rows_b)
        self.assertEqual(summary_a, summary_b)
        self.assertEqual(summary_a["num_cases"], 8)
        self.assertIn("success_rate", summary_a)


if __name__ == "__main__":
    unittest.main()
