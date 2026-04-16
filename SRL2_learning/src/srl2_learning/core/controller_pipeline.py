"""教学版最小主流程控制器。

这个文件在整条链路里的位置：
- 它是当前学习版里最接近“主流程总装器”的文件。
- 它把规划、状态更新、动力学接口、映射和执行记录串成一条可观察的离线链路。

为什么需要它：
- 新手单看某个模块时，往往不知道它什么时候被调用、结果会流向哪里。
- 这个文件让你从一个入口走完整条链路。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from ..adapters.mock_actuator import MockActuatorRecorder
from .constants import DEFAULT_COMMAND_DOF, DEFAULT_CONTROL_PERIOD, DEFAULT_JOINT_DOF
from .dynamics_service import DynamicsComputationResult, OfflineDynamicsService
from .joint_motor_mapper import JointMotorCommand, joint_to_motor_commands
from .state_tracking import DesiredMotionTracker
from .trajectory_planner import LinearInterpolationTrajectoryPlanner


@dataclass(slots=True)
class OfflinePipelineStep:
    """主链路单步快照。

    属性说明：
    - `step_index` (`int`)：当前是第几步。
    - `desired_joint_position` (`list[float]`)：这一拍的期望关节位置。
    - `desired_joint_velocity` (`list[float]`)：这一拍的期望关节速度。
    - `desired_joint_acceleration` (`list[float]`)：这一拍的期望关节加速度。
    - `mapped_motor_command` (`JointMotorCommand`)：映射后的电机命令。
    - `dynamics_result` (`DynamicsComputationResult`)：动力学接口结果。
    - `is_finished` (`bool`)：这一步之后轨迹是否结束。
    """

    step_index: int
    desired_joint_position: list[float]
    desired_joint_velocity: list[float]
    desired_joint_acceleration: list[float]
    mapped_motor_command: JointMotorCommand
    dynamics_result: DynamicsComputationResult
    is_finished: bool


class LearningRobotController:
    """教学版最小控制链路。

    属性说明：
    - `control_period` (`float`)：控制周期。
    - `joint_dof` (`int`)：主关节自由度数量。
    - `command_dof` (`int`)：命令向量长度。
    - `planner` (`LinearInterpolationTrajectoryPlanner`)：轨迹规划器。
    - `desired_tracker` (`DesiredMotionTracker`)：期望状态跟踪器。
    - `dynamics_service` (`OfflineDynamicsService`)：动力学服务。
    - `actuator` (`MockActuatorRecorder`)：执行记录器。
    """

    def __init__(
        self,
        control_period: float = DEFAULT_CONTROL_PERIOD,
        joint_dof: int = DEFAULT_JOINT_DOF,
        command_dof: int = DEFAULT_COMMAND_DOF,
    ) -> None:
        self.control_period = control_period
        self.joint_dof = joint_dof
        self.command_dof = command_dof

        self.planner = LinearInterpolationTrajectoryPlanner()
        self.planner.initialize(dof_count=command_dof, control_period=control_period)

        self.desired_tracker = DesiredMotionTracker(control_period=control_period)
        self.dynamics_service = OfflineDynamicsService(joint_dof=joint_dof)
        self.actuator = MockActuatorRecorder()

    def load_reference_model(
        self,
        model_path: str,
        end_link_name: str = "Link6",
        tool_offset: Sequence[float] = (0.0, 0.0, 0.0),
    ) -> None:
        """登记参考模型。

        参数：
        - `model_path` (`str`)：模型路径。
        - `end_link_name` (`str`)：末端链节名。
        - `tool_offset` (`Sequence[float]`)：工具偏移量。

        返回值：
        - 无。

        例子：
        ```python
        controller.load_reference_model(
            model_path="model/HitLimb_up.urdf",
            end_link_name="Link6",
            tool_offset=(0.0, 0.0, 0.0),
        )
        ```
        """
        self.dynamics_service.load_model(
            reference_model_path=model_path,
            end_link_name=end_link_name,
            tool_offset=tool_offset,
        )

    def run_joint_space_motion(
        self,
        target_joint_command: Sequence[float],
        start_joint_command: Sequence[float] | None = None,
        max_velocity: float = 0.5,
        max_acceleration: float = 0.5,
    ) -> list[OfflinePipelineStep]:
        """运行一段关节空间离线运动。

        参数：
        - `target_joint_command` (`Sequence[float]`)：目标关节命令。
        - `start_joint_command` (`Sequence[float] | None`)：起点命令；如果不传，默认从全零开始。
        - `max_velocity` (`float`)：轨迹最大速度。
        - `max_acceleration` (`float`)：轨迹最大加速度。

        返回值：
        - `list[OfflinePipelineStep]`：
          整段运动的逐步快照列表。

        最小例子：
        ```python
        steps = controller.run_joint_space_motion(
            start_joint_command=[0.0] * 7,
            target_joint_command=[0.2, -0.1, 0.15, 0.0, 0.0, 0.0, 0.0],
            max_velocity=0.5,
            max_acceleration=0.5,
        )
        ```
        """

        # 先检查输入维度是否正确
        if len(target_joint_command) != self.command_dof:
            raise ValueError("目标命令维度与控制器配置不一致。")
        
        # 如果没给起点，就默认从全零开始
        if start_joint_command is None:
            start_joint_command = [0.0] * self.command_dof
        if len(start_joint_command) != self.command_dof:
            raise ValueError("起点命令维度与控制器配置不一致。")

        # 把“从起点到终点”的任务交给规划器登记
        self.planner.plan_joint_position(
            current_position=start_joint_command,
            target_position=target_joint_command,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
        )

        # 重置状态跟踪器
        self.desired_tracker.reset(self.command_dof)

        # 把起点同步给动力学层
        self.dynamics_service.set_joint_positions(start_joint_command[: self.joint_dof])

        # 把起点同步给动力学层
        self.actuator.reset()

        # 准备一个空列表，用来保存每一拍结果的快照
        step_results: list[OfflinePipelineStep] = []
        is_first_step = True
        step_index = 0

        # 进入循环
        while True:

            #向规划器取出这一拍的目标位置
            planner_step = self.planner.step()

            # 用状态跟踪器对这个位置做状态补全，得到速度和加速度
            desired_state = self.desired_tracker.update(
                planner_step.position,
                first_time=is_first_step,
            )
            is_first_step = False

            # 只取前6维送进动力学层
            joint_side_position = desired_state.position[: self.joint_dof]
            joint_side_velocity = desired_state.velocity[: self.joint_dof]
            joint_side_acceleration = desired_state.acceleration[: self.joint_dof]

            # 从动力学层拿回关节力矩（当前教学版其实是零力矩占位）
            dynamics_result = self.dynamics_service.inverse_dynamics(
                joint_positions=joint_side_position,
                joint_velocities=joint_side_velocity,
                joint_accelerations=joint_side_acceleration,
            )
            # 把6维关节力矩补成7维
            # 这里把关节侧力矩补齐到完整命令长度。
            # 原因是：完整命令向量长度通常不只包含主关节，还可能预留其他执行通道。
            full_joint_torque = dynamics_result.joint_torques + [0.0] * (self.command_dof - self.joint_dof)

            # 把关节位置/速度/力矩翻译成电机位置/速度/力矩
            mapped_command = joint_to_motor_commands(
                joint_positions=desired_state.position,
                joint_velocities=desired_state.velocity,
                joint_torques=full_joint_torque,
            )

            # 把这组电机命令记录下来
            self.actuator.record(
                step_index=step_index,
                motor_positions=mapped_command.motor_positions,
                motor_velocities=mapped_command.motor_velocities,
                motor_torques=mapped_command.motor_torques,
            )
            # 把这一拍的完整快照保存起来
            step_results.append(
                OfflinePipelineStep(
                    step_index=step_index,
                    desired_joint_position=desired_state.position.copy(),
                    desired_joint_velocity=desired_state.velocity.copy(),
                    desired_joint_acceleration=desired_state.acceleration.copy(),
                    mapped_motor_command=mapped_command,
                    dynamics_result=dynamics_result,
                    is_finished=planner_step.is_finished,
                )
            )

            # 如果轨迹结束，就退出循环
            if planner_step.is_finished:
                break
            # 拍数++
            step_index += 1
        # 返回整段轨迹的所有单步快照
        return step_results
