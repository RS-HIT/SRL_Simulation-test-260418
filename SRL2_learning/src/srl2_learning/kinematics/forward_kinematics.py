"""四坐标系正运动学接口。

当前统一使用四个坐标系：
- world frame：世界坐标系
- base frame：基座坐标系
- flange frame：末端法兰坐标系
- tool frame：工具坐标系

这里的职责就是：
- 给一组关节角
- 明确算出这四个坐标系里和实验最相关的位姿信息
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .model_context import MujocoKinematicModelContext
from .tool_frame import ToolFrameConfig


@dataclass(slots=True)
class ForwardKinematicsResult:
    """四坐标系正运动学结果。"""

    world_origin: list[float]
    base_position: list[float]
    base_rotation_matrix: np.ndarray
    flange_position: list[float]
    flange_rotation_matrix: np.ndarray
    tool_position: list[float]
    tool_rotation_matrix: np.ndarray
    tool_offset_world: list[float]
    reference_name: str
    reference_body_name: str
    reference_site_name: str | None
    reference_kind: str
    used_site: bool
    tcp_position: list[float]
    tcp_rotation_matrix: np.ndarray
    tcp_offset_world: list[float]


def _read_flange_pose(context: MujocoKinematicModelContext) -> tuple[np.ndarray, np.ndarray, str, bool]:
    """读取当前法兰参考的世界位姿。"""
    if context.reference_site_name is not None:
        site_id = mujoco.mj_name2id(context.model, mujoco.mjtObj.mjOBJ_SITE, context.reference_site_name)
        if site_id >= 0:
            position = np.array(context.data.site_xpos[site_id], dtype=float)
            rotation = np.array(context.data.site_xmat[site_id], dtype=float).reshape(3, 3)
            return position, rotation, context.reference_site_name, True

    body_id = mujoco.mj_name2id(context.model, mujoco.mjtObj.mjOBJ_BODY, context.reference_body_name)
    position = np.array(context.data.xpos[body_id], dtype=float)
    rotation = np.array(context.data.xmat[body_id], dtype=float).reshape(3, 3)
    return position, rotation, context.reference_body_name, False


def compute_forward_kinematics(
    context: MujocoKinematicModelContext,
    joint_positions: list[float],
    tool_frame: ToolFrameConfig | None = None,
) -> ForwardKinematicsResult:
    """根据关节角计算 world/base/flange/tool 的位姿关系。"""
    context.set_joint_positions(joint_positions)
    flange_position, flange_rotation, reference_name, used_site = _read_flange_pose(context)

    tool_position = flange_position.copy()
    tool_rotation = flange_rotation.copy()
    tool_offset_world = np.zeros(3, dtype=float)
    if tool_frame is not None and tool_frame.enabled:
        tool_offset_world = flange_rotation @ tool_frame.translation_vector
        tool_position = flange_position + tool_offset_world
        tool_rotation = flange_rotation @ tool_frame.rotation_matrix

    return ForwardKinematicsResult(
        world_origin=context.world_frame_origin.copy(),
        base_position=context.base_position.copy(),
        base_rotation_matrix=context.base_rotation_matrix.copy(),
        flange_position=flange_position.tolist(),
        flange_rotation_matrix=flange_rotation.copy(),
        tool_position=tool_position.tolist(),
        tool_rotation_matrix=tool_rotation.copy(),
        tool_offset_world=tool_offset_world.tolist(),
        reference_name=reference_name,
        reference_body_name=context.reference_body_name,
        reference_site_name=context.reference_site_name,
        reference_kind="site" if used_site else "body",
        used_site=used_site,
        tcp_position=tool_position.tolist(),
        tcp_rotation_matrix=tool_rotation.copy(),
        tcp_offset_world=tool_offset_world.tolist(),
    )
