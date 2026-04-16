from __future__ import annotations

import unittest
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.core.controller_pipeline import LearningRobotController


class PipelineTests(unittest.TestCase):
    def test_pipeline_can_run_minimal_joint_motion(self) -> None:
        controller = LearningRobotController()
        controller.load_reference_model(
            model_path=str(PROJECT_ROOT.parent / "SRL2" / "model" / "HitLimb_up.urdf"),
            end_link_name="Link6",
            tool_offset=(0.0, 0.0, 0.0),
        )

        results = controller.run_joint_space_motion(
            start_joint_command=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            target_joint_command=[0.2, -0.1, 0.15, 0.0, 0.0, 0.0, 0.0],
            max_velocity=0.5,
            max_acceleration=0.5,
        )

        self.assertGreater(len(results), 0)
        self.assertEqual(len(results), len(controller.actuator.commands))
        self.assertEqual(results[-1].desired_joint_position, [0.2, -0.1, 0.15, 0.0, 0.0, 0.0, 0.0])
        self.assertFalse(results[-1].dynamics_result.confirmed)


if __name__ == "__main__":
    unittest.main()

