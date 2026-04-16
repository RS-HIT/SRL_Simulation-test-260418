from __future__ import annotations

import unittest
from pathlib import Path
import sys

import mujoco

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.mujoco_replay import SRL2MujocoModelAdapter, SRL2QposWriter


class MuJoCoReplayTests(unittest.TestCase):
    def test_runtime_meshless_model_can_load(self) -> None:
        adapter = SRL2MujocoModelAdapter(PROJECT_ROOT.parent / "SRL2" / "model" / "Rsras1.xml")
        model, model_info = adapter.load_model()

        self.assertEqual(model.nq, 29)
        self.assertEqual(model.njnt, 17)
        self.assertFalse(model_info.mesh_enabled)
        self.assertTrue(model_info.meshless_playback_mode)
        self.assertIn("robot_tip_r", model_info.site_names)
        self.assertIn("arm_tip_r", model_info.site_names)

    def test_qpos_writer_matches_original_main_cpp_layout(self) -> None:
        qpos = SRL2QposWriter().build_qpos([1, 2, 3, 4, 5, 6])

        self.assertEqual(len(qpos), 29)
        self.assertEqual(qpos[0:3], [0.0, 0.0, 0.0])
        self.assertEqual(qpos[3:7], [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(qpos[7:13], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(qpos[13:19], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(qpos[19:23], [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(qpos[23], 0.0)
        self.assertEqual(qpos[24:28], [1.0, 0.0, 0.0, 0.0])
        self.assertEqual(qpos[28], 0.0)

    def test_offscreen_renderer_can_render_one_frame(self) -> None:
        adapter = SRL2MujocoModelAdapter(PROJECT_ROOT.parent / "SRL2" / "model" / "Rsras1.xml")
        model, _ = adapter.load_model()
        data = mujoco.MjData(model)
        SRL2QposWriter().apply_frame(data, [0.1, -0.1, 0.2, 0.0, 0.0, 0.0])
        mujoco.mj_forward(model, data)

        renderer = mujoco.Renderer(model, width=160, height=120)
        renderer.update_scene(data)
        image = renderer.render()
        renderer.close()

        self.assertEqual(image.shape[2], 3)
        self.assertGreater(image.shape[0], 0)
        self.assertGreater(image.shape[1], 0)


if __name__ == "__main__":
    unittest.main()
