"""教学版动力学接口层。

这个文件在整条链路里的位置：
- 它位于“状态更新”之后、“关节到电机映射”之前。

这个文件有什么用：
- 它让学习版保留一个与原项目相似的“模型/动力学入口”。
- 即使当前没有真实 RBDL，我们也能先看懂主流程结构。

当前是否为教学替身：
- 是。
- 当前返回的是明确标注为“未确认”的占位结果。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


@dataclass(slots=True)
class DynamicsComputationResult:
    """动力学计算结果容器。

    属性说明：
    - `joint_torques` (`list[float]`)：
      关节力矩结果。
      当前教学版返回的是零列表。
    - `end_position` (`list[float] | None`)：
      末端位置。当前未实现时为 `None`。
    - `end_orientation_rpy` (`list[float] | None`)：
      末端姿态，通常用滚转/俯仰/偏航表示。
    - `confirmed` (`bool`)：
      结果是否已被验证为接近原版真实行为。
      当前教学版占位结果为 `False`。
    - `note` (`str`)：
      对当前结果可信度和限制的文字说明。
    """

    joint_torques: list[float]
    end_position: list[float] | None
    end_orientation_rpy: list[float] | None
    confirmed: bool
    note: str


class OfflineDynamicsService:
    """教学版动力学服务。

    属性说明：
    - `joint_dof` (`int`)：关节自由度数量。
    - `reference_model_path` (`Path | None`)：参考模型路径。
    - `end_link_name` (`str | None`)：末端链节名称。
    - `tool_offset` (`tuple[float, float, float] | None`)：工具偏移量。
    - `current_joint_positions` (`list[float]`)：当前缓存的关节位置。
    """

    def __init__(self, joint_dof: int) -> None:
        self.joint_dof = joint_dof
        self.reference_model_path: Path | None = None
        self.end_link_name: str | None = None
        self.tool_offset: tuple[float, float, float] | None = None
        self.current_joint_positions = [0.0] * joint_dof

    def load_model(
        self,
        reference_model_path: str,
        end_link_name: str,
        tool_offset: Sequence[float],
    ) -> None:
        """登记参考模型信息。

        参数：
        - `reference_model_path` (`str`)：模型文件路径。
        - `end_link_name` (`str`)：末端链节名。
        - `tool_offset` (`Sequence[float]`)：工具偏移。

        返回值：
        - 无。

        例子：
        ```python
        service.load_model(
            "model/HitLimb_up.urdf",
            "Link6",
            (0.0, 0.0, 0.0),
        )
        ```
        """
        model_path = Path(reference_model_path)
        self.reference_model_path = model_path
        self.end_link_name = end_link_name
        self.tool_offset = tuple(float(value) for value in tool_offset)

    def set_joint_positions(self, joint_positions: Sequence[float]) -> None:
        """同步当前关节位置缓存。"""
        if len(joint_positions) != self.joint_dof:
            raise ValueError("关节数量与动力学服务配置不一致。")
        self.current_joint_positions = [float(value) for value in joint_positions]

    def get_joint_positions(self) -> list[float]:
        """返回当前缓存的关节位置副本。"""
        return self.current_joint_positions.copy()

    def inverse_dynamics(
        self,
        joint_positions: Sequence[float],
        joint_velocities: Sequence[float],
        joint_accelerations: Sequence[float],
    ) -> DynamicsComputationResult:
        """执行教学版逆动力学接口。

        参数：
        - `joint_positions` (`Sequence[float]`)：关节位置。
        - `joint_velocities` (`Sequence[float]`)：关节速度。
        - `joint_accelerations` (`Sequence[float]`)：关节加速度。

        返回值：
        - `DynamicsComputationResult`

        例子：
        ```python
        result = service.inverse_dynamics(
            joint_positions=[0.0] * 6,
            joint_velocities=[0.0] * 6,
            joint_accelerations=[0.0] * 6,
        )
        # result.confirmed 当前为 False
        ```
        """
        if len(joint_positions) != self.joint_dof:
            raise ValueError("关节位置维度不正确。")
        if len(joint_velocities) != self.joint_dof:
            raise ValueError("关节速度维度不正确。")
        if len(joint_accelerations) != self.joint_dof:
            raise ValueError("关节加速度维度不正确。")

        self.current_joint_positions = [float(value) for value in joint_positions]
        return DynamicsComputationResult(
            joint_torques=[0.0] * self.joint_dof,
            end_position=None,
            end_orientation_rpy=None,
            confirmed=False,
            note=(
                "当前环境未接入 RBDL。此处仅保留接口形状，并返回零力矩占位结果；"
                "真实逆动力学行为待在 Linux + RBDL 环境下验证。"
            ),
        )
