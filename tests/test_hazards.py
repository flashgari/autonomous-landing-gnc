import unittest

from landing_gnc.hazards import point_clearance_m, select_safe_target
from landing_gnc.models import HazardZone


class HazardTests(unittest.TestCase):
    def test_selector_chooses_nearest_clear_candidate(self):
        hazards = (HazardZone(-4.0, 4.0),)
        target = select_safe_target(18.0, (-16.0, 0.0, 12.0), hazards)
        self.assertEqual(target, 12.0)
        self.assertGreaterEqual(point_clearance_m(target, hazards), 3.0)


if __name__ == "__main__":
    unittest.main()
