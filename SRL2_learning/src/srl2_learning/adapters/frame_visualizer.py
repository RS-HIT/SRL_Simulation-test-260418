"""MuJoCo viewer 中的坐标系与目标点可视化层。

这个模块只负责 viewer 里的调试显示：
- world / base / flange / tool 四组坐标系
- 目标点 marker
- 对应名字标签

它不参与 IK、轨迹规划、CSV 导出。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Sequence

import mujoco
import numpy as np

from ..kinematics.forward_kinematics import compute_forward_kinematics
from ..kinematics.model_context import MujocoKinematicModelContext
from ..kinematics.tool_frame import ToolFrameConfig


FRAME_AXIS_COLORS = (
    np.array([1.0, 0.15, 0.15, 1.0], dtype=float),
    np.array([0.15, 1.0, 0.15, 1.0], dtype=float),
    np.array([0.15, 0.35, 1.0, 1.0], dtype=float),
)
TARGET_POINT_RGBA = np.array([1.0, 0.2, 0.85, 1.0], dtype=float)


@dataclass(slots=True)
class FrameVisualizationConfig:
    """viewer 中调试图层的配置。"""

    show_world_frame: bool = True
    show_base_frame: bool = True
    show_flange_frame: bool = True
    show_tool_frame: bool = True
    show_target_point: bool = True
    show_frame_names: bool = True
    frame_axis_length: float = 0.12
    frame_axis_radius: float = 0.008
    frame_origin_radius: float = 0.014
    target_marker_radius: float = 0.018
    world_frame_origin: list[float] | None = None
    print_frame_pose_each_step: bool = False
    print_frame_pose_every_n_steps: int = 10


@dataclass(slots=True)
class ViewerCameraConfig:
    """MuJoCo free camera 的初始视角配置。"""

    lookat: list[float] | None = None
    distance: float | None = None
    azimuth: float | None = None
    elevation: float | None = None


def apply_viewer_camera(
    camera: mujoco.MjvCamera,
    camera_config: ViewerCameraConfig | None,
) -> None:
    """把配置里的初始视角写到 MuJoCo viewer camera。

    这些参数都只影响“窗口刚打开时看向哪里、离多远、从哪个角度看”。
    它们不改变机器人模型本身的位置，也不改变 world/base/flange/tool 的定义。
    """
    if camera_config is None:
        return
    if camera_config.lookat is not None:
        camera.lookat[:] = [float(value) for value in camera_config.lookat]
    if camera_config.distance is not None:
        camera.distance = float(camera_config.distance)
    if camera_config.azimuth is not None:
        camera.azimuth = float(camera_config.azimuth)
    if camera_config.elevation is not None:
        camera.elevation = float(camera_config.elevation)


def _reset_user_scene(user_scn: mujoco.MjvScene) -> None:
    user_scn.ngeom = 0


def _append_sphere(
    user_scn: mujoco.MjvScene,
    position: np.ndarray,
    radius: float,
    rgba: np.ndarray,
    label: str = "",
) -> None:
    if user_scn.ngeom >= len(user_scn.geoms):
        return
    geom = user_scn.geoms[user_scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, radius, radius], dtype=float),
        np.asarray(position, dtype=float),
        np.eye(3, dtype=float).reshape(-1),
        rgba,
    )
    geom.label = label
    user_scn.ngeom += 1


def _append_axis_arrow(
    user_scn: mujoco.MjvScene,
    from_position: np.ndarray,
    to_position: np.ndarray,
    axis_radius: float,
    rgba: np.ndarray,
) -> None:
    if user_scn.ngeom >= len(user_scn.geoms):
        return
    geom = user_scn.geoms[user_scn.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_ARROW,
        np.array([axis_radius, axis_radius, axis_radius], dtype=float),
        np.zeros(3, dtype=float),
        np.eye(3, dtype=float).reshape(-1),
        rgba,
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_ARROW,
        float(axis_radius),
        np.asarray(from_position, dtype=float),
        np.asarray(to_position, dtype=float),
    )
    user_scn.ngeom += 1


def draw_frame_markers(
    user_scn: mujoco.MjvScene,
    frame_name: str,
    origin: np.ndarray,
    rotation_matrix: np.ndarray,
    axis_length: float,
    axis_radius: float,
    origin_radius: float,
    show_frame_names: bool,
) -> None:
    """为一组坐标系画原点球、三根轴和可选名字。"""
    _append_sphere(
        user_scn=user_scn,
        position=origin,
        radius=origin_radius,
        rgba=np.array([1.0, 0.9, 0.1, 1.0], dtype=float),
        label=frame_name if show_frame_names else "",
    )
    for axis_index in range(3):
        direction = rotation_matrix[:, axis_index]
        _append_axis_arrow(
            user_scn=user_scn,
            from_position=origin,
            to_position=origin + direction * float(axis_length),
            axis_radius=axis_radius,
            rgba=FRAME_AXIS_COLORS[axis_index],
        )


def draw_target_markers(
    user_scn: mujoco.MjvScene,
    target_positions: Sequence[Sequence[float]],
    marker_radius: float,
    show_names: bool,
) -> None:
    """把目标点画成单独的球 marker。"""
    for index, target_position in enumerate(target_positions):
        _append_sphere(
            user_scn=user_scn,
            position=np.asarray(target_position, dtype=float),
            radius=marker_radius,
            rgba=TARGET_POINT_RGBA,
            label=f"target_{index + 1}" if show_names else "",
        )


def update_frame_visualization(
    user_scn: mujoco.MjvScene,
    frame_config: FrameVisualizationConfig,
    world_position: np.ndarray,
    base_position: np.ndarray,
    base_rotation: np.ndarray,
    flange_position: np.ndarray,
    flange_rotation: np.ndarray,
    tool_position: np.ndarray,
    tool_rotation: np.ndarray,
    target_positions: Sequence[Sequence[float]] | None = None,
) -> None:
    """按当前配置刷新四个坐标系和可选目标点。"""
    _reset_user_scene(user_scn)

    if frame_config.show_world_frame:
        draw_frame_markers(
            user_scn=user_scn,
            frame_name="world",
            origin=world_position,
            rotation_matrix=np.eye(3, dtype=float),
            axis_length=frame_config.frame_axis_length,
            axis_radius=frame_config.frame_axis_radius,
            origin_radius=frame_config.frame_origin_radius,
            show_frame_names=frame_config.show_frame_names,
        )
    if frame_config.show_base_frame:
        draw_frame_markers(
            user_scn=user_scn,
            frame_name="base",
            origin=base_position,
            rotation_matrix=base_rotation,
            axis_length=frame_config.frame_axis_length,
            axis_radius=frame_config.frame_axis_radius,
            origin_radius=frame_config.frame_origin_radius,
            show_frame_names=frame_config.show_frame_names,
        )
    if frame_config.show_flange_frame:
        draw_frame_markers(
            user_scn=user_scn,
            frame_name="flange",
            origin=flange_position,
            rotation_matrix=flange_rotation,
            axis_length=frame_config.frame_axis_length,
            axis_radius=frame_config.frame_axis_radius,
            origin_radius=frame_config.frame_origin_radius,
            show_frame_names=frame_config.show_frame_names,
        )
    if frame_config.show_tool_frame:
        draw_frame_markers(
            user_scn=user_scn,
            frame_name="tool",
            origin=tool_position,
            rotation_matrix=tool_rotation,
            axis_length=frame_config.frame_axis_length,
            axis_radius=frame_config.frame_axis_radius,
            origin_radius=frame_config.frame_origin_radius,
            show_frame_names=frame_config.show_frame_names,
        )
    if frame_config.show_target_point and target_positions:
        draw_target_markers(
            user_scn=user_scn,
            target_positions=target_positions,
            marker_radius=frame_config.target_marker_radius,
            show_names=frame_config.show_frame_names,
        )


def _should_print_pose(frame_config: FrameVisualizationConfig, step_index: int) -> bool:
    if frame_config.print_frame_pose_each_step:
        return True
    return step_index % max(int(frame_config.print_frame_pose_every_n_steps), 1) == 0


def replay_cartesian_experiment_in_viewer(
    model_context: MujocoKinematicModelContext,
    joint_trajectory: list[list[float]],
    control_period: float,
    playback_speed: float,
    frame_config: FrameVisualizationConfig,
    camera_config: ViewerCameraConfig | None = None,
    tool_frame: ToolFrameConfig | None = None,
    startup_pause_seconds: float = 0.8,
    keep_open_after_replay: bool = True,
    target_position: list[float] | None = None,
    target_positions: Sequence[Sequence[float]] | None = None,
) -> None:
    """在 MuJoCo viewer 中播放实验，并叠加 frame 与目标点。"""
    import mujoco.viewer as viewer

    overlay_target_positions = list(target_positions or ([] if target_position is None else [target_position]))

    data = mujoco.MjData(model_context.model)
    with viewer.launch_passive(model_context.model, data) as viewer_handle:
        apply_viewer_camera(viewer_handle.cam, camera_config)
        viewer_handle.sync()
        if startup_pause_seconds > 0:
            time.sleep(startup_pause_seconds)

        for step_index, joint_positions in enumerate(joint_trajectory):
            if not viewer_handle.is_running():
                return
            data.qpos[:] = joint_positions
            mujoco.mj_forward(model_context.model, data)
            model_context.set_joint_positions(joint_positions)

            fk_result = compute_forward_kinematics(model_context, joint_positions, tool_frame)
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
                target_positions=overlay_target_positions,
            )
            viewer_handle.sync()

            if _should_print_pose(frame_config, step_index):
                print(
                    f"[frame step {step_index}] "
                    f"base={fk_result.base_position}, "
                    f"flange={fk_result.flange_position}, "
                    f"tool={fk_result.tool_position}, "
                    f"tool_offset_world={fk_result.tool_offset_world}, "
                    f"targets={overlay_target_positions}"
                )

            time.sleep(float(control_period) / max(float(playback_speed), 1e-6))

        if keep_open_after_replay:
            while viewer_handle.is_running():
                viewer_handle.sync()
                time.sleep(0.03)
