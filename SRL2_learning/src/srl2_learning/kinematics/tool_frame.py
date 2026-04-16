"""工具坐标系配置。

这里专门负责“工具坐标系偏移常量”的定义和读取。

给新手的直白解释：
- 法兰坐标系：机械臂本体最后一级参考点
- 工具坐标系：真正拿来做任务和比较误差的点
- 工具坐标系 = 法兰坐标系 + 工具偏移

当前你最该改的工具偏移常量位置就是：
- `configs/tool_frame_config.json`

字段说明：
- `tool_translation_xyz`：工具原点相对法兰原点的平移，单位米
- `tool_rotation_rpy`：工具坐标系相对法兰坐标系的旋转，单位弧度

兼容说明：
- 为兼容旧配置，仍支持旧字段 `translation_xyz` / `rotation_rpy`
- 读取时优先使用新字段
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def euler_rpy_to_rotation_matrix(rotation_rpy: list[float]) -> np.ndarray:
    """把 roll / pitch / yaw 欧拉角转换成旋转矩阵。"""
    roll, pitch, yaw = [float(value) for value in rotation_rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rotation_z @ rotation_y @ rotation_x


@dataclass(slots=True)
class ToolFrameConfig:
    """工具坐标系配置。"""

    enabled: bool
    label: str
    tool_translation_xyz: list[float]
    tool_rotation_rpy: list[float]

    @property
    def translation_xyz(self) -> list[float]:
        """兼容旧代码使用的别名。"""
        return self.tool_translation_xyz

    @property
    def rotation_rpy(self) -> list[float]:
        """兼容旧代码使用的别名。"""
        return self.tool_rotation_rpy

    @property
    def translation_vector(self) -> np.ndarray:
        """工具坐标系平移向量。"""
        return np.array(self.tool_translation_xyz, dtype=float)

    @property
    def rotation_matrix(self) -> np.ndarray:
        """工具坐标系旋转矩阵。"""
        return euler_rpy_to_rotation_matrix(self.tool_rotation_rpy)


def load_tool_frame_config(config_path: str | Path) -> ToolFrameConfig:
    """读取工具坐标系配置文件。"""
    payload = json.loads(Path(config_path).read_text(encoding="utf-8"))
    tool_translation_xyz = payload.get("tool_translation_xyz", payload.get("translation_xyz", [0.0, 0.0, 0.0]))
    tool_rotation_rpy = payload.get("tool_rotation_rpy", payload.get("rotation_rpy", [0.0, 0.0, 0.0]))
    return ToolFrameConfig(
        enabled=bool(payload.get("enabled", False)),
        label=str(payload.get("label", "default_tool_frame")),
        tool_translation_xyz=[float(value) for value in tool_translation_xyz],
        tool_rotation_rpy=[float(value) for value in tool_rotation_rpy],
    )
