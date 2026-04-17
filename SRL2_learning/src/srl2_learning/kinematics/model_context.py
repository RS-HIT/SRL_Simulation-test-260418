"""MuJoCo 运动学实验上下文。

这个文件负责统一提供四坐标系实验需要的基础事实：
- world frame 原点
- base frame 常量
- flange frame 参考对象
- joint limit 约束
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from ..adapters.urdf_real_mesh_mujoco import URDFRealMeshPreparationAdapter, load_real_mesh_mujoco_model
from .tool_frame import euler_rpy_to_rotation_matrix


def _build_joint_limit_overrides(
    joint_limit_overrides_radians: dict[str, list[float]] | None = None,
    joint_limit_overrides_degrees: dict[str, list[float]] | None = None,
) -> dict[str, tuple[float, float]]:
    """统一整理关节限位覆盖表。

    优先级：
    1. 弧度配置
    2. 角度配置
    """
    resolved: dict[str, tuple[float, float]] = {}
    if joint_limit_overrides_degrees:
        for joint_name, limits in joint_limit_overrides_degrees.items():
            if len(limits) != 2:
                raise ValueError(f"关节 {joint_name} 的角度限位必须是 [min_deg, max_deg]")
            resolved[str(joint_name)] = (math.radians(float(limits[0])), math.radians(float(limits[1])))
    if joint_limit_overrides_radians:
        for joint_name, limits in joint_limit_overrides_radians.items():
            if len(limits) != 2:
                raise ValueError(f"关节 {joint_name} 的弧度限位必须是 [min_rad, max_rad]")
            resolved[str(joint_name)] = (float(limits[0]), float(limits[1]))
    return resolved


@dataclass(slots=True)
class MujocoKinematicModelContext:
    """MuJoCo 运动学模型上下文。"""

    model: mujoco.MjModel
    data: mujoco.MjData
    scene_model_path: Path
    joint_names: list[str]
    body_names: list[str]
    site_names: list[str]
    joint_qpos_indices: list[int]
    joint_lower_limits: np.ndarray
    joint_upper_limits: np.ndarray
    world_frame_origin: list[float]
    base_position: list[float]
    base_euler: list[float]
    base_rotation_matrix: np.ndarray
    reference_body_name: str
    reference_site_name: str | None

    def set_joint_positions(self, joint_positions: list[float]) -> None:
        """把关节角写进 `qpos` 并刷新前向运动学。"""
        if len(joint_positions) != len(self.joint_qpos_indices):
            raise ValueError("输入关节角长度与模型自由度不一致。")
        for qpos_index, joint_value in zip(self.joint_qpos_indices, joint_positions, strict=True):
            self.data.qpos[qpos_index] = float(joint_value)
        mujoco.mj_forward(self.model, self.data)

    def clamp_joint_positions(self, joint_positions: np.ndarray) -> np.ndarray:
        """按当前生效的关节限位裁剪关节角。"""
        return np.clip(joint_positions, self.joint_lower_limits, self.joint_upper_limits)

    def has_body(self, body_name: str) -> bool:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name) >= 0

    def has_site(self, site_name: str) -> bool:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name) >= 0

    def describe_reference(self) -> dict[str, object]:
        reference_kind = "site" if self.reference_site_name and self.has_site(self.reference_site_name) else "body"
        reference_name = self.reference_site_name if reference_kind == "site" else self.reference_body_name
        joint_limits = {
            joint_name: [float(self.joint_lower_limits[index]), float(self.joint_upper_limits[index])]
            for index, joint_name in enumerate(self.joint_names)
        }
        return {
            "reference_kind": reference_kind,
            "reference_name": reference_name,
            "reference_body_name": self.reference_body_name,
            "reference_site_name": self.reference_site_name,
            "scene_model_path": str(self.scene_model_path),
            "world_frame_origin": self.world_frame_origin,
            "base_position": self.base_position,
            "base_euler": self.base_euler,
            "joint_limits_radians": joint_limits,
        }


def load_real_mesh_model_context(
    real_mesh_config_path: str | Path,
    reference_body_name: str = "Link3",
    reference_site_name: str | None = None,
    joint_limit_overrides_radians: dict[str, list[float]] | None = None,
    joint_limit_overrides_degrees: dict[str, list[float]] | None = None,
) -> MujocoKinematicModelContext:
    """从真实网格配置加载默认运动学实验模型。"""
    config_path = Path(real_mesh_config_path).resolve()
    config = json.loads(config_path.read_text(encoding="utf-8"))

    source_urdf_path = (config_path.parent / config["source_urdf_path"]).resolve()
    processed_model_root = (config_path.parent.parent / config["processed_model_root"]).resolve()

    adapter = URDFRealMeshPreparationAdapter(
        source_urdf_path=source_urdf_path,
        processed_model_root=processed_model_root,
    )
    preparation_result = adapter.prepare_assets(
        mesh_face_limits=config["mesh_face_limits"],
        base_position=config["base_position"],
        base_euler=config["base_euler"],
        ground_height=config["ground_height"],
        add_ground_plane=config.get("add_ground_plane", False),
        default_joint_limit=tuple(config["default_joint_limit"]),
        force_rebuild=False,
    )

    scene_model_path = preparation_result.runtime_mjcf_path
    if not scene_model_path.exists():
        scene_model_path = preparation_result.runtime_urdf_path

    model = load_real_mesh_mujoco_model(scene_model_path)
    data = mujoco.MjData(model)

    joint_names: list[str] = []
    body_names: list[str] = []
    site_names: list[str] = []
    joint_qpos_indices: list[int] = []
    joint_lower_limits: list[float] = []
    joint_upper_limits: list[float] = []

    resolved_overrides = _build_joint_limit_overrides(
        joint_limit_overrides_radians=joint_limit_overrides_radians,
        joint_limit_overrides_degrees=joint_limit_overrides_degrees,
    )

    for joint_id in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if joint_name is None:
            continue
        joint_names.append(joint_name)
        joint_qpos_indices.append(int(model.jnt_qposadr[joint_id]))

        lower_limit = float(model.jnt_range[joint_id][0])
        upper_limit = float(model.jnt_range[joint_id][1])
        if joint_name in resolved_overrides:
            lower_limit, upper_limit = resolved_overrides[joint_name]
        if lower_limit > upper_limit:
            raise ValueError(f"关节 {joint_name} 的限位下界不能大于上界。")
        joint_lower_limits.append(lower_limit)
        joint_upper_limits.append(upper_limit)

    for unknown_joint_name in sorted(set(resolved_overrides) - set(joint_names)):
        raise ValueError(f"关节限位配置里出现了模型中不存在的关节: {unknown_joint_name}")

    for body_id in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if body_name is not None:
            body_names.append(body_name)

    for site_id in range(model.nsite):
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, site_id)
        if site_name is not None:
            site_names.append(site_name)

    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, reference_body_name) < 0:
        raise ValueError(f"找不到参考 body: {reference_body_name}")
    if reference_site_name is not None and mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, reference_site_name) < 0:
        reference_site_name = None

    base_position = [float(value) for value in config["base_position"]]
    base_euler = [float(value) for value in config["base_euler"]]
    return MujocoKinematicModelContext(
        model=model,
        data=data,
        scene_model_path=scene_model_path,
        joint_names=joint_names,
        body_names=body_names,
        site_names=site_names,
        joint_qpos_indices=joint_qpos_indices,
        joint_lower_limits=np.array(joint_lower_limits, dtype=float),
        joint_upper_limits=np.array(joint_upper_limits, dtype=float),
        world_frame_origin=[0.0, 0.0, 0.0],
        base_position=base_position,
        base_euler=base_euler,
        base_rotation_matrix=euler_rpy_to_rotation_matrix(base_euler),
        reference_body_name=reference_body_name,
        reference_site_name=reference_site_name,
    )
