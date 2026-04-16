from __future__ import annotations

import unittest
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.core.state_tracking import DesiredMotionTracker, LowPassFilter


class StateTrackingTests(unittest.TestCase):
    def test_desired_motion_tracker_uses_discrete_difference(self) -> None:
        tracker = DesiredMotionTracker(control_period=0.005)
        tracker.reset(3)

        first = tracker.update([0.0, 0.0, 0.0], first_time=True)
        second = tracker.update([0.005, -0.01, 0.015], first_time=False)

        self.assertEqual(first.velocity, [0.0, 0.0, 0.0])
        self.assertAlmostEqual(second.velocity[0], 1.0)
        self.assertAlmostEqual(second.velocity[1], -2.0)
        self.assertAlmostEqual(second.velocity[2], 3.0)

    def test_low_pass_filter_matches_reference_formula(self) -> None:
        low_pass_filter = LowPassFilter(alpha=0.2)
        low_pass_filter.reset(3)

        filtered = low_pass_filter.update([10.0, 0.0, -10.0])
        self.assertEqual(filtered, [2.0, 0.0, -2.0])


if __name__ == "__main__":
    unittest.main()

