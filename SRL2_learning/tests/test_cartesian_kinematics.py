from __future__ import annotations

import subprocess
import sys
import unittest
from pathlib import Path

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.cartesian_visualization import (
    export_cartesian_plots,
    export_cartesian_summary_csv,
    export_cartesian_tracking_csv,
)
from srl2_learning.adapters.frame_visualizer import FrameVisualizationConfig, update_frame_visualization
from srl2_learning.experiments.cartesian_target_experiment import (
    run_multi_target_experiment,
    run_single_target_experiment,
    run_tcp_sensitivity_experiment,
)
from srl2_learning.kinematics.forward_kinematics import compute_forward_kinematics
from srl2_learning.kinematics.model_context import load_real_mesh_model_context
from srl2_learning.kinematics.tool_frame import ToolFrameConfig, load_tool_frame_config


class CartesianKinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.real_mesh_config_path = PROJECT_ROOT / "configs" / "urdf_real_mesh_demo_config.json"
        cls.model_context = load_real_mesh_model_context(cls.real_mesh_config_path)

    def test_forward_kinematics_tool_offset_matches_rotated_translation(self) -> None:
        joint_positions = [0.1, -0.2, 0.3]
        no_tool_result = compute_forward_kinematics(self.model_context, joint_positions, tool_frame=None)
        tool_config = ToolFrameConfig(
            enabled=True,
            label="test_tool",
            tool_translation_xyz=[0.05, 0.0, 0.0],
            tool_rotation_rpy=[0.0, 0.0, 0.0],
        )
        with_tool_result = compute_forward_kinematics(self.model_context, joint_positions, tool_frame=tool_config)
        expected_delta = no_tool_result.flange_rotation_matrix @ np.asarray(tool_config.tool_translation_xyz, dtype=float)
        actual_delta = np.asarray(with_tool_result.tool_position) - np.asarray(no_tool_result.tool_position)
        self.assertTrue(np.allclose(actual_delta, expected_delta, atol=1e-6))
        self.assertEqual(with_tool_result.reference_kind, "body")
        self.assertTrue(np.allclose(with_tool_result.base_position, self.model_context.base_position, atol=1e-9))

    def test_inverse_kinematics_reaches_known_point(self) -> None:
        reference_joint_positions = [0.25, -0.45, 0.35]
        target_position = compute_forward_kinematics(self.model_context, reference_joint_positions).tool_position
        result = run_single_target_experiment(
            model_context=self.model_context,
            target_position=target_position,
            initial_joint_positions=[0.0, 0.0, 0.0],
            control_period=0.02,
            max_velocity=0.6,
            max_acceleration=0.6,
            ik_max_iterations=200,
            ik_tolerance=0.002,
            ik_step_size=0.8,
            ik_damping=0.01,
        )
        self.assertTrue(result.summaries[0].converged)
        self.assertLess(result.summaries[0].final_tool_error_norm, 0.01)

    def test_inverse_kinematics_unreachable_target_keeps_best_result(self) -> None:
        result = run_single_target_experiment(
            model_context=self.model_context,
            target_position=[10.0, 10.0, 10.0],
            initial_joint_positions=[0.0, 0.0, 0.0],
            control_period=0.02,
            max_velocity=0.6,
            max_acceleration=0.6,
            ik_max_iterations=8,
            ik_tolerance=1e-6,
            ik_step_size=0.5,
            ik_damping=0.01,
        )
        self.assertFalse(result.summaries[0].converged)
        self.assertGreater(result.summaries[0].final_tool_error_norm, 0.1)
        self.assertEqual(len(result.joint_trajectory[-1]), 3)

    def test_tool_frame_config_loads_from_json(self) -> None:
        tool_config_path = PROJECT_ROOT / "configs" / "tool_frame_config.json"
        tool_config = load_tool_frame_config(tool_config_path)
        self.assertEqual(tool_config.tool_translation_xyz, [0.0, 0.0, 0.0])
        self.assertEqual(tool_config.tool_rotation_rpy, [0.0, 0.0, 0.0])
        self.assertEqual(tool_config.translation_xyz, [0.0, 0.0, 0.0])

    def test_multi_target_and_visualization_outputs(self) -> None:
        output_directory = PROJECT_ROOT / "logs" / "test_cartesian_multi"
        result = run_multi_target_experiment(
            model_context=self.model_context,
            target_points=[[0.106287, -0.416270, 1.132398], [0.186802, -0.441837, 1.064071]],
            initial_joint_positions=[0.0, 0.0, 0.0],
            control_period=0.02,
            max_velocity=0.6,
            max_acceleration=0.6,
            ik_max_iterations=180,
            ik_tolerance=0.003,
            ik_step_size=0.8,
            ik_damping=0.01,
            hold_steps=5,
        )
        export_cartesian_tracking_csv(output_directory / "cartesian_tracking.csv", result.samples)
        export_cartesian_summary_csv(output_directory / "cartesian_target_summary.csv", result.summaries)
        export_cartesian_plots(output_directory, result.samples, "测试用末端实验")
        csv_text = (output_directory / "cartesian_tracking.csv").read_text(encoding="utf-8-sig")
        self.assertIn("base_x", csv_text)
        self.assertIn("flange_x", csv_text)
        self.assertIn("tool_x", csv_text)
        self.assertIn("tool_error_norm", csv_text)
        self.assertTrue((output_directory / "cartesian_position_tracking.png").exists())
        self.assertTrue((output_directory / "cartesian_error.png").exists())
        self.assertTrue((output_directory / "cartesian_trajectory_xy.png").exists())

    def test_tool_sensitivity_runs_multiple_cases(self) -> None:
        results = run_tcp_sensitivity_experiment(
            model_context=self.model_context,
            target_position=[0.106287, -0.416270, 1.132398],
            initial_joint_positions=[0.0, 0.0, 0.0],
            tcp_cases=[
                ToolFrameConfig(True, "case_a", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                ToolFrameConfig(True, "case_b", [0.05, 0.0, 0.0], [0.0, 0.0, 0.0]),
            ],
            control_period=0.02,
            max_velocity=0.6,
            max_acceleration=0.6,
            ik_max_iterations=150,
            ik_tolerance=0.003,
            ik_step_size=0.8,
            ik_damping=0.01,
            hold_steps=5,
        )
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0].summaries[0].tool_label, "case_a")
        self.assertEqual(results[1].summaries[0].tool_label, "case_b")

    def test_frame_visualizer_writes_markers_into_user_scene(self) -> None:
        scene = mujoco.MjvScene(self.model_context.model, 200)
        frame_config = FrameVisualizationConfig()
        update_frame_visualization(
            user_scn=scene,
            frame_config=frame_config,
            world_position=np.array([0.0, 0.0, 0.0], dtype=float),
            base_position=np.array([0.0, 0.0, 0.65], dtype=float),
            base_rotation=np.eye(3, dtype=float),
            flange_position=np.array([0.1, 0.2, 0.3], dtype=float),
            flange_rotation=np.eye(3, dtype=float),
            tool_position=np.array([0.15, 0.25, 0.35], dtype=float),
            tool_rotation=np.eye(3, dtype=float),
        )
        self.assertGreaterEqual(scene.ngeom, 16)

    def test_inspector_script_runs(self) -> None:
        result = subprocess.run(
            [sys.executable, str(PROJECT_ROOT / "scripts" / "inspect_end_effector_frames.py")],
            cwd=PROJECT_ROOT.parent,
            capture_output=True,
            text=True,
            check=True,
        )
        self.assertIn("world frame 原点", result.stdout)
        self.assertIn("base frame 原点", result.stdout)
        self.assertIn("tool frame 原点", result.stdout)
        self.assertIn("误差比较点", result.stdout)


if __name__ == "__main__":
    unittest.main()
