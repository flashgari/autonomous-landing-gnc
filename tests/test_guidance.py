import unittest

from landing_gnc.guidance import vertical_velocity_reference
from landing_gnc.models import Guidance


class GuidanceTests(unittest.TestCase):
    def test_vertical_velocity_reference_softens_near_ground(self):
        guidance = Guidance()
        high = vertical_velocity_reference(800.0, guidance)
        low = vertical_velocity_reference(5.0, guidance)
        self.assertLess(high, low)
        self.assertLess(abs(low), 10.0)


if __name__ == "__main__":
    unittest.main()

