from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

import mujoco
import trimesh

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.urdf_mujoco_demo import (
    build_joint_trajectory,
    collect_urdf_replay_frames,
)
from srl2_learning.adapters.urdf_real_mesh_mujoco import (
    URDFRealMeshPreparationAdapter,
    load_real_mesh_mujoco_model,
)


class URDFRealMeshMujocoTests(unittest.TestCase):
    def setUp(self) -> None:
        self.urdf_path = PROJECT_ROOT.parent / "urdf_0.1.4.2" / "urdf" / "urdf_0.1.4.2.urdf"
        self.processed_root = PROJECT_ROOT / "logs" / "test_real_mesh_assets"
        self.adapter = URDFRealMeshPreparationAdapter(
            source_urdf_path=self.urdf_path,
            processed_model_root=self.processed_root,
        )

    def test_preprocess_rewrites_paths_and_decimates_large_meshes(self) -> None:
        result = self.adapter.prepare_assets(
            mesh_face_limits={"base.STL": None, "Link1.STL": 150000, "Link2.STL": 150000, "Link3.STL": None},
            base_position=[0.0, 0.0, 0.65],
            base_euler=[0.0, 0.0, 0.0],
            ground_height=0.0,
            add_ground_plane=False,
            default_joint_limit=(-3.14159, 3.14159),
            force_rebuild=True,
        )
        runtime_urdf_text = result.runtime_urdf_path.read_text(encoding="utf-8")
        self.assertNotIn("package://", runtime_urdf_text)
        self.assertTrue(result.manifest_path.exists())
        self.assertTrue(result.runtime_mjcf_path.exists())

        link1_mesh = trimesh.load_mesh(result.processed_mesh_directory / "Link1.STL", force="mesh")
        link2_mesh = trimesh.load_mesh(result.processed_mesh_directory / "Link2.STL", force="mesh")
        self.assertLessEqual(len(link1_mesh.faces), 200000)
        self.assertLessEqual(len(link2_mesh.faces), 200000)

        runtime_mjcf_text = result.runtime_mjcf_path.read_text(encoding="utf-8")
        self.assertIn('type="skybox"', runtime_mjcf_text)
        self.assertIn('name="groundplane_texture"', runtime_mjcf_text)
        self.assertIn('name="groundplane_material"', runtime_mjcf_text)
        self.assertIn('name="scene_floor"', runtime_mjcf_text)
        self.assertIn('name="scene_light"', runtime_mjcf_text)

    def test_real_mesh_model_can_load(self) -> None:
        result = self.adapter.prepare_assets(force_rebuild=False)
        model = load_real_mesh_mujoco_model(result.runtime_urdf_path)
        self.assertEqual(model.nq, 3)
        self.assertEqual(model.njnt, 3)
        self.assertGreater(model.ngeom, 0)

    def test_real_mesh_replay_final_qpos_matches_target(self) -> None:
        result = self.adapter.prepare_assets(force_rebuild=False)
        model = load_real_mesh_mujoco_model(result.runtime_urdf_path)
        target = [0.45, -0.7, 0.9]
        trajectory = build_joint_trajectory(
            start_joint_position=[0.0, 0.0, 0.0],
            target_joint_position=target,
            control_period=0.02,
            max_velocity=0.6,
            max_acceleration=0.6,
        )
        frames = collect_urdf_replay_frames(
            model=model,
            joint_positions=trajectory,
            control_period=0.02,
            observed_site_names=["Link3_site"],
        )
        self.assertGreater(len(frames), 1)
        self.assertEqual(frames[-1].qpos, target)


if __name__ == "__main__":
    unittest.main()
