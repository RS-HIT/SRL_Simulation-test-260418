"""教学版轨迹规划器。

这个文件在整条链路里的位置：
- 它位于“输入目标”之后、“状态更新”之前。
- 它负责把“最终想去哪”拆成“这一拍先走到哪”。

为什么需要这一层：
- 机械臂不能从起点瞬移到终点。
- 如果没有中间轨迹，后面的状态更新和控制都会过于生硬。

当前是否为教学替身：
- 是。
- 原项目依赖 Reflexxes Type II 做实时轨迹规划。
- 这里使用线性插值，只保留“初始化 -> 设目标 -> 逐拍输出”的学习骨架。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil
from typing import Sequence


@dataclass(slots=True)
class PlannerStepResult:
    """单步轨迹输出。

    属性说明：
    - `position` (`list[float]`)：
      当前这一拍的期望位置。
    - `velocity` (`list[float]`)：
      当前这一拍的期望速度。
    - `acceleration` (`list[float]`)：
      当前这一拍的期望加速度。
      学习版里当前固定为 0.0 列表，用于保留接口形状。
    - `is_finished` (`bool`)：
      当前轨迹是否已经到终点。
      例子：如果这是最后一步，就会是 `True`。
    """

    position: list[float]
    velocity: list[float]
    acceleration: list[float]
    is_finished: bool


class LinearInterpolationTrajectoryPlanner:
    """用线性插值实现的教学版规划器。

    这个类为什么存在：
    - 它让新手先理解“轨迹规划”的最核心概念：把大动作拆成很多小步。

    它在主流程里承担什么角色：
    - 它是“路径生成者”。

    属性说明：
    - `dof_count` (`int`)：
      自由度数量，也就是每个命令向量有多少个数字。
    - `control_period` (`float`)：
      控制周期，单位秒。
    - `start_position` (`list[float]`)：
      当前轨迹段的起点。
    - `target_position` (`list[float]`)：
      当前轨迹段的终点。
    - `current_step_index` (`int`)：
      已经走到第几步，从 0 开始。
    - `total_step_count` (`int`)：
      这段轨迹总共需要多少步。
    - `max_velocity` (`float`)：
      用来估算总步数的最大速度。
    - `max_acceleration` (`float`)：
      当前教学版暂只保留接口，不用于真实加速度规划。
    """

    def __init__(self) -> None:
        self.dof_count = 0
        self.control_period = 0.0
        self.start_position: list[float] = []
        self.target_position: list[float] = []
        self.current_step_index = 0
        self.total_step_count = 0
        self.max_velocity = 0.0
        self.max_acceleration = 0.0

    def initialize(self, dof_count: int, control_period: float) -> None:
        """在第一次规划前配置规划器。

        参数：
        - `dof_count` (`int`)：向量长度。
        - `control_period` (`float`)：控制周期，单位秒。

        返回值：
        - 无。

        最小例子：
        ```python
        planner = LinearInterpolationTrajectoryPlanner()
        planner.initialize(dof_count=7, control_period=0.005)
        ```
        """
        self.dof_count = dof_count
        self.control_period = control_period

    def plan_joint_position(
        self,
        current_position: Sequence[float],
        target_position: Sequence[float],
        max_velocity: float = -1.0,
        max_acceleration: float = -1.0,
    ) -> None:
        """登记一段新的关节空间轨迹。

        参数：
        - `current_position` (`Sequence[float]`)：
          轨迹起点。
        - `target_position` (`Sequence[float]`)：
          轨迹终点。
        - `max_velocity` (`float`)：
          最大速度。默认 `-1.0` 表示使用教学版默认值。
        - `max_acceleration` (`float`)：
          最大加速度。当前教学版主要保留接口形状。

        返回值：
        - 无。结果保存在对象内部，后续通过 `step()` 逐步取出。

        具体例子：
        ```python
        planner.plan_joint_position(
            current_position=[0.0, 0.0],
            target_position=[1.0, 0.5],
            max_velocity=0.5,
            max_acceleration=0.5,
        )
        ```
        """
        if self.dof_count <= 0 or self.control_period <= 0:
            raise ValueError("请先调用 initialize 设置自由度和控制周期。")

        if len(current_position) != self.dof_count or len(target_position) != self.dof_count:
            raise ValueError("输入向量长度与规划器自由度不一致。")

        self.start_position = [float(value) for value in current_position]
        self.target_position = [float(value) for value in target_position]
        self.current_step_index = 0
        self.max_velocity = 10.0 if max_velocity == -1 else float(max_velocity)
        self.max_acceleration = 10.0 if max_acceleration == -1 else float(max_acceleration)

        max_displacement = max(
            abs(target - current)
            for current, target in zip(self.start_position, self.target_position, strict=True)
        )
        if max_displacement == 0:
            self.total_step_count = 1
            return

        estimated_total_time = max_displacement / max(self.max_velocity, 1e-6)
        self.total_step_count = max(1, ceil(estimated_total_time / self.control_period))

    def step(self) -> PlannerStepResult:
        """取出当前控制周期对应的一步规划结果。

        返回值：
        - `PlannerStepResult`：
          包含当前位置、速度、加速度和是否结束。

        最小例子：
        ```python
        planner = LinearInterpolationTrajectoryPlanner()
        planner.initialize(2, 0.1)
        planner.plan_joint_position([0.0, 0.0], [1.0, 0.0], 0.5, 0.5)
        result = planner.step()
        ```

        新手容易误解的点：
        - `step()` 每调用一次，只返回“一拍”的结果，不是整段轨迹。
        """
        if self.total_step_count == 0:
            raise RuntimeError("请先调用 plan_joint_position。")

        progress = min(1.0, (self.current_step_index + 1) / self.total_step_count)
        position = [
            start + (target - start) * progress
            for start, target in zip(self.start_position, self.target_position, strict=True)
        ]

        # 这里也用了 zip(..., strict=True)。
        # 大白话：按同一下标把起点和终点配对，然后逐项算当前步的位置和速度。
        velocity = [
            (target - start) / (self.total_step_count * self.control_period)
            for start, target in zip(self.start_position, self.target_position, strict=True)
        ]
        acceleration = [0.0] * self.dof_count

        self.current_step_index += 1
        is_finished = self.current_step_index >= self.total_step_count
        if is_finished:
            position = self.target_position.copy()

        return PlannerStepResult(
            position=position,
            velocity=velocity,
            acceleration=acceleration,
            is_finished=is_finished,
        )
