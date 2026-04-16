from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.urdf_mujoco_demo import (
    URDFKinematicChainAdapter,
    build_joint_trajectory,
    collect_urdf_replay_frames,
)


class URDFMujocoDemoTests(unittest.TestCase):
    def setUp(self) -> None:
        self.urdf_path = PROJECT_ROOT.parent / "urdf_0.1.4.2" / "urdf" / "urdf_0.1.4.2.urdf"

    def test_urdf_runtime_model_can_build(self) -> None:
        adapter = URDFKinematicChainAdapter(source_urdf_path=self.urdf_path)
        result = adapter.build_runtime_model(
            output_directory=PROJECT_ROOT / "logs" / "test_urdf_build",
            base_height=0.65,
            end_effector_offset=[0.0, -0.22, 0.0],
            default_limit=(-3.14159, 3.14159),
            timestep=0.02,
            dump_runtime_xml=False,
        )
        self.assertEqual(result.model.nq, 3)
        self.assertEqual(len(result.joint_specs), 3)
        self.assertIn("tool_site", result.site_names)

    def test_urdf_replay_final_qpos_matches_target(self) -> None:
        adapter = URDFKinematicChainAdapter(source_urdf_path=self.urdf_path)
        result = adapter.build_runtime_model(
            output_directory=PROJECT_ROOT / "logs" / "test_urdf_build",
            base_height=0.65,
            end_effector_offset=[0.0, -0.22, 0.0],
            default_limit=(-3.14159, 3.14159),
            timestep=0.02,
            dump_runtime_xml=False,
        )
        target = [0.45, -0.7, 0.9]
        trajectory = build_joint_trajectory(
            start_joint_position=[0.0, 0.0, 0.0],
            target_joint_position=target,
            control_period=0.02,
            max_velocity=0.6,
            max_acceleration=0.6,
        )
        frames = collect_urdf_replay_frames(
            model=result.model,
            joint_positions=trajectory,
            control_period=0.02,
            observed_site_names=["tool_site"],
        )
        self.assertGreater(len(frames), 1)
        self.assertEqual(frames[-1].qpos, target)


if __name__ == "__main__":
    unittest.main()
