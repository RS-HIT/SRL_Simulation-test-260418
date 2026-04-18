"""关节角度拖动调试器。

用途：
- 实时显示每个关节当前角度
- 允许用滑块直接拖动每个关节
- 不做速度积分，不做动力学推进
- 你松手后，关节就停在当前角度

这适合用来观察：
- 当前初始姿态是否合理
- 每个关节的大致工作区间
- 应该给每个关节设置什么限位
"""

from __future__ import annotations

import json
import math
import sys
import threading
import time
import tkinter as tk
import traceback
from pathlib import Path
from tkinter import ttk

import mujoco
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.frame_visualizer import (
    FrameVisualizationConfig,
    ViewerCameraConfig,
    apply_viewer_camera,
    update_frame_visualization,
)
from srl2_learning.kinematics.forward_kinematics import compute_forward_kinematics
from srl2_learning.kinematics.model_context import load_real_mesh_model_context
from srl2_learning.kinematics.tool_frame import load_tool_frame_config


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


class JointAngleTunerApp:
    def __init__(self) -> None:
        self.experiment_config_path = PROJECT_ROOT / "configs" / "cartesian_target_experiment.json"
        self.experiment_config = _load_json(self.experiment_config_path)
        self.real_mesh_config_path = (self.experiment_config_path.parent / self.experiment_config["real_mesh_config_path"]).resolve()
        self.tool_frame_config_path = (self.experiment_config_path.parent / self.experiment_config["tool_frame_config_path"]).resolve()
        self.tool_frame = load_tool_frame_config(self.tool_frame_config_path)
        # 这四个参数专门控制“viewer 刚打开时相机站在哪里、看向哪里”。
        # 想调整初始视窗，不要改 world/base/flange/tool 的定义，优先改这里对应的配置项。
        self.viewer_camera_config = ViewerCameraConfig(
            lookat=[float(value) for value in self.experiment_config.get("viewer_camera_lookat", [0.0, 0.0, 0.85])],
            distance=float(self.experiment_config.get("viewer_camera_distance", 2.6)),
            azimuth=float(self.experiment_config.get("viewer_camera_azimuth", 135.0)),
            elevation=float(self.experiment_config.get("viewer_camera_elevation", -18.0)),
        )

        self.model_context = load_real_mesh_model_context(
            real_mesh_config_path=self.real_mesh_config_path,
            reference_body_name=self.experiment_config.get("flange_reference_body", self.experiment_config["reference_body_name"]),
            reference_site_name=self.experiment_config.get("flange_reference_site", self.experiment_config["reference_site_name"]),
            joint_limit_overrides_radians=self.experiment_config.get("joint_position_limits_radians"),
            joint_limit_overrides_degrees=self.experiment_config.get("joint_position_limits_degrees"),
        )
        # 这个调试器只做“把关节直接摆到某个角度并观察姿态”。
        # 它不需要真实接触约束，也不需要求解接触力。
        # 禁掉 contact 可以避免真实网格和地板在首帧就生成大量约束，
        # 从而触发 MuJoCo 的 arena memory 警告并导致 viewer 提前退出。
        self.model_context.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)

        initial = np.asarray(self.experiment_config["initial_joint_positions"], dtype=float)
        self.current_joint_positions = self.model_context.clamp_joint_positions(initial)
        self.observed_min_positions = self.current_joint_positions.copy()
        self.observed_max_positions = self.current_joint_positions.copy()
        self.latest_flange_position_text = "-"
        self.latest_tool_position_text = "-"

        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.viewer_thread = threading.Thread(target=self._viewer_loop, daemon=True)
        self.viewer_exception_message: str | None = None

        self.root = tk.Tk()
        self.root.title("SRL2_learning 关节角度拖动调试器")
        self.root.geometry("720x620")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.degree_vars: list[tk.DoubleVar] = []
        self.current_labels: list[ttk.Label] = []
        self.observed_labels: list[ttk.Label] = []
        self.flange_label: ttk.Label | None = None
        self.tool_label: ttk.Label | None = None
        self.status_label: ttk.Label | None = None

        self._build_ui()

    def _build_ui(self) -> None:
        header = ttk.Label(
            self.root,
            text=(
                "这个调试器不会按速度继续运动。\n"
                "它只会把滑块对应的关节角直接写进 qpos，再刷新 MuJoCo 前向运动学。"
            ),
            justify="left",
        )
        header.pack(anchor="w", padx=12, pady=(12, 8))

        config_info = ttk.Label(
            self.root,
            text=(
                f"关节限位来源: {self.experiment_config_path}\n"
                f"基座偏移来源: {self.real_mesh_config_path}\n"
                f"工具偏移来源: {self.tool_frame_config_path}"
            ),
            justify="left",
        )
        config_info.pack(anchor="w", padx=12, pady=(0, 10))

        sliders_frame = ttk.Frame(self.root)
        sliders_frame.pack(fill="both", expand=True, padx=12, pady=8)

        for index, joint_name in enumerate(self.model_context.joint_names):
            lower_rad = float(self.model_context.joint_lower_limits[index])
            upper_rad = float(self.model_context.joint_upper_limits[index])
            lower_deg = math.degrees(lower_rad)
            upper_deg = math.degrees(upper_rad)
            current_deg = math.degrees(float(self.current_joint_positions[index]))

            row = ttk.LabelFrame(sliders_frame, text=f"{joint_name}")
            row.pack(fill="x", padx=4, pady=6)

            range_label = ttk.Label(
                row,
                text=f"允许范围: {lower_rad:.4f} rad ~ {upper_rad:.4f} rad    ({lower_deg:.1f}° ~ {upper_deg:.1f}°)",
                justify="left",
            )
            range_label.pack(anchor="w", padx=8, pady=(6, 2))

            degree_var = tk.DoubleVar(value=current_deg)
            self.degree_vars.append(degree_var)

            scale = tk.Scale(
                row,
                from_=lower_deg,
                to=upper_deg,
                resolution=0.1,
                orient=tk.HORIZONTAL,
                length=520,
                variable=degree_var,
                command=lambda _value, joint_index=index: self._on_slider_changed(joint_index),
            )
            scale.pack(anchor="w", padx=8, pady=2)

            current_label = ttk.Label(row, text="")
            current_label.pack(anchor="w", padx=8, pady=2)
            self.current_labels.append(current_label)

            observed_label = ttk.Label(row, text="")
            observed_label.pack(anchor="w", padx=8, pady=(0, 6))
            self.observed_labels.append(observed_label)

        pose_frame = ttk.LabelFrame(self.root, text="当前末端信息")
        pose_frame.pack(fill="x", padx=12, pady=8)

        self.flange_label = ttk.Label(pose_frame, text="flange: -", justify="left")
        self.flange_label.pack(anchor="w", padx=8, pady=(6, 2))
        self.tool_label = ttk.Label(pose_frame, text="tool: -", justify="left")
        self.tool_label.pack(anchor="w", padx=8, pady=(0, 6))

        self.status_label = ttk.Label(
            self.root,
            text="viewer 状态: 启动中",
            justify="left",
        )
        self.status_label.pack(anchor="w", padx=12, pady=(0, 8))

        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill="x", padx=12, pady=(0, 12))

        ttk.Button(button_frame, text="打印当前关节角", command=self._print_current_joint_positions).pack(side="left", padx=(0, 8))
        ttk.Button(button_frame, text="打印观察到的限位建议", command=self._print_observed_limits).pack(side="left", padx=(0, 8))
        ttk.Button(button_frame, text="重置观察范围", command=self._reset_observed_limits).pack(side="left")

        self._refresh_ui_labels()

    def _on_slider_changed(self, joint_index: int) -> None:
        with self.state_lock:
            new_radian_value = math.radians(self.degree_vars[joint_index].get())
            self.current_joint_positions[joint_index] = new_radian_value
            self.current_joint_positions = self.model_context.clamp_joint_positions(self.current_joint_positions)
            self.observed_min_positions = np.minimum(self.observed_min_positions, self.current_joint_positions)
            self.observed_max_positions = np.maximum(self.observed_max_positions, self.current_joint_positions)
        self._refresh_ui_labels()

    def _refresh_ui_labels(self) -> None:
        with self.state_lock:
            current_positions = self.current_joint_positions.copy()
            observed_min = self.observed_min_positions.copy()
            observed_max = self.observed_max_positions.copy()
            flange_position_text = self.latest_flange_position_text
            tool_position_text = self.latest_tool_position_text

        for index, joint_name in enumerate(self.model_context.joint_names):
            current_rad = float(current_positions[index])
            current_deg = math.degrees(current_rad)
            observed_min_deg = math.degrees(float(observed_min[index]))
            observed_max_deg = math.degrees(float(observed_max[index]))
            self.current_labels[index].config(
                text=f"当前角度: {current_rad:.4f} rad    {current_deg:.2f}°"
            )
            self.observed_labels[index].config(
                text=(
                    f"已观察范围: {float(observed_min[index]):.4f} ~ {float(observed_max[index]):.4f} rad    "
                    f"({observed_min_deg:.2f}° ~ {observed_max_deg:.2f}°)"
                )
            )

        if self.flange_label is not None:
            self.flange_label.config(text=f"flange: {flange_position_text}")
        if self.tool_label is not None:
            self.tool_label.config(text=f"tool: {tool_position_text}")
        if self.status_label is not None:
            status_text = "viewer 状态: 已关闭" if self.stop_event.is_set() else "viewer 状态: 运行中"
            if self.viewer_exception_message:
                status_text = f"viewer 状态: 异常退出\n{self.viewer_exception_message}"
            self.status_label.config(text=status_text)

        if not self.stop_event.is_set():
            self.root.after(100, self._refresh_ui_labels)

    def _print_current_joint_positions(self) -> None:
        with self.state_lock:
            current_positions = self.current_joint_positions.copy()
        payload = {
            joint_name: {
                "radians": float(current_positions[index]),
                "degrees": float(math.degrees(current_positions[index])),
            }
            for index, joint_name in enumerate(self.model_context.joint_names)
        }
        print("当前关节角：")
        print(json.dumps(payload, ensure_ascii=False, indent=2))

    def _print_observed_limits(self) -> None:
        with self.state_lock:
            observed_min = self.observed_min_positions.copy()
            observed_max = self.observed_max_positions.copy()
        radians_payload = {
            joint_name: [float(observed_min[index]), float(observed_max[index])]
            for index, joint_name in enumerate(self.model_context.joint_names)
        }
        degrees_payload = {
            joint_name: [float(math.degrees(observed_min[index])), float(math.degrees(observed_max[index]))]
            for index, joint_name in enumerate(self.model_context.joint_names)
        }
        print("建议写入配置的关节限位（弧度）：")
        print(json.dumps({"joint_position_limits_radians": radians_payload}, ensure_ascii=False, indent=2))
        print("建议写入配置的关节限位（角度）：")
        print(json.dumps({"joint_position_limits_degrees": degrees_payload}, ensure_ascii=False, indent=2))

    def _reset_observed_limits(self) -> None:
        with self.state_lock:
            self.observed_min_positions = self.current_joint_positions.copy()
            self.observed_max_positions = self.current_joint_positions.copy()
        self._refresh_ui_labels()

    def _viewer_loop(self) -> None:
        import mujoco.viewer as viewer

        frame_config = FrameVisualizationConfig(
            show_world_frame=True,
            show_base_frame=True,
            show_flange_frame=True,
            show_tool_frame=True,
            show_target_point=False,
            show_frame_names=True,
        )
        try:
            with viewer.launch_passive(self.model_context.model, self.model_context.data) as viewer_handle:
                apply_viewer_camera(viewer_handle.cam, self.viewer_camera_config)
                viewer_handle.sync()
                while viewer_handle.is_running() and not self.stop_event.is_set():
                    with self.state_lock:
                        current_positions = self.current_joint_positions.copy()
                    self.model_context.set_joint_positions(current_positions.tolist())
                    fk_result = compute_forward_kinematics(
                        context=self.model_context,
                        joint_positions=current_positions.tolist(),
                        tool_frame=self.tool_frame if self.experiment_config.get("use_tool_frame", False) else None,
                    )
                    with self.state_lock:
                        self.latest_flange_position_text = str(fk_result.flange_position)
                        self.latest_tool_position_text = str(fk_result.tool_position)
                    update_frame_visualization(
                        user_scn=viewer_handle.user_scn,
                        frame_config=frame_config,
                        world_position=np.asarray(fk_result.world_origin, dtype=float),
                        base_position=np.asarray(fk_result.base_position, dtype=float),
                        base_rotation=np.asarray(fk_result.base_rotation_matrix, dtype=float),
                        flange_position=np.asarray(fk_result.flange_position, dtype=float),
                        flange_rotation=np.asarray(fk_result.flange_rotation_matrix, dtype=float),
                        tool_position=np.asarray(fk_result.tool_position, dtype=float),
                        tool_rotation=np.asarray(fk_result.tool_rotation_matrix, dtype=float),
                        target_positions=None,
                    )
                    viewer_handle.sync()
                    time.sleep(0.02)
        except Exception as exc:
            self.viewer_exception_message = f"{type(exc).__name__}: {exc}"
            print("关节角度拖动调试器的 viewer 线程异常退出：")
            traceback.print_exc()
        finally:
            self.stop_event.set()

    def _on_close(self) -> None:
        self.stop_event.set()
        self.root.destroy()

    def run(self) -> None:
        print("启动关节角度拖动调试器。")
        print(f"当前生效的关节限位来自: {self.experiment_config_path}")
        print("提示：拖动滑块时，关节会立刻停在滑块位置，不会按速度继续运动。")
        self.viewer_thread.start()
        self.root.mainloop()
        self.stop_event.set()


def main() -> None:
    app = JointAngleTunerApp()
    app.run()


if __name__ == "__main__":
    main()
