"""教学版执行记录器。

这个文件在整条链路里的位置：
- 它位于主流程最后一层。
- 真实工程里这一层会把命令发给硬件；学习版里它只负责记录。
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(slots=True)
class MockActuatorCommand:
    """一次离线执行记录。

    属性说明：
    - `step_index` (`int`)：第几步。
    - `motor_positions` (`list[float]`)：电机位置命令。
    - `motor_velocities` (`list[float]`)：电机速度命令。
    - `motor_torques` (`list[float]`)：电机力矩命令。
    """

    step_index: int
    motor_positions: list[float]
    motor_velocities: list[float]
    motor_torques: list[float]


class MockActuatorRecorder:
    """教学版执行层记录器。

    属性说明：
    - `commands` (`list[MockActuatorCommand]`)：
      已记录的全部命令。
      例子：跑完一段 10 步轨迹后，这里通常会有 10 条记录。
    """

    def __init__(self) -> None:
        self.commands: list[MockActuatorCommand] = []

    def reset(self) -> None:
        """清空历史命令记录。

        返回值：
        - 无。
        """
        self.commands.clear()

    def record(
        self,
        step_index: int,
        motor_positions: list[float],
        motor_velocities: list[float],
        motor_torques: list[float],
    ) -> None:
        """记录当前这一拍准备发送的电机命令。

        参数：
        - `step_index` (`int`)：当前步编号。
        - `motor_positions` (`list[float]`)：电机位置命令。
        - `motor_velocities` (`list[float]`)：电机速度命令。
        - `motor_torques` (`list[float]`)：电机力矩命令。

        返回值：
        - 无。

        例子：
        ```python
        recorder.record(
            step_index=0,
            motor_positions=[0.0] * 7,
            motor_velocities=[0.0] * 7,
            motor_torques=[0.0] * 7,
        )
        ```
        """
        self.commands.append(
            MockActuatorCommand(
                step_index=step_index,
                motor_positions=motor_positions.copy(),
                motor_velocities=motor_velocities.copy(),
                motor_torques=motor_torques.copy(),
            )
        )
