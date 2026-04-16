"""关节命令与电机命令之间的翻译层。

这个文件在整条链路里的位置：
- 它位于“关节侧状态已经准备好”和“执行层准备接收命令”之间。
- 前面的模块给出的还是关节空间的位置、速度、力矩。
- 到了这里，这些量才会被翻译成电机空间命令。

这个文件解决什么问题：
- 原项目里，关节和电机不是简单的一一对应。
- 尤其前 3 个通道存在耦合映射，如果不先做翻译，执行层拿到的命令就不对。

如果没有它，项目会卡在哪：
- 我们会停在“我知道关节想怎么动”，但还不知道“电机到底要怎么动”。

当前是否为教学替身：
- 不完全是替身。
- 前 3 个通道的核心映射公式保留自原项目的可确认逻辑。
- 但后 4 个通道的速度和力矩处理，保留了原项目当前代码的实际结果，并明确标记为历史风险点。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from .constants import (
    DEFAULT_COMMAND_DOF,
    JOINT_1_REDUCTION_RATIO,
    JOINT_2_REDUCTION_RATIO,
    JOINT_3_REDUCTION_RATIO,
)


def _as_command_vector(values: Sequence[float], expected_length: int = DEFAULT_COMMAND_DOF) -> list[float]:
    """把输入整理成固定长度的浮点列表。

    这个函数是干什么的：
    - 把元组、列表等输入统一转成 `list[float]`。
    - 同时检查长度是否符合当前命令向量要求。

    它在整个项目流程里处于哪一步：
    - 它不是主流程大步骤，而是映射函数开始前的“门卫”。
    - 只有输入长度正确，后面的映射公式才有意义。

    参数：
    - `values` (`Sequence[float]`)：
      任何“像列表一样可遍历”的数字序列。
      例子：`[0, 1, 2]`、`(0.1, 0.2, 0.3)`。
    - `expected_length` (`int`)：
      期望长度，默认是 `DEFAULT_COMMAND_DOF`，也就是 7。

    返回值：
    - `list[float]`：
      转换后的浮点列表。

    最小例子：
    ```python
    _as_command_vector((1, 2, 3), expected_length=3)
    # 返回 [1.0, 2.0, 3.0]
    ```
    """
    result = list(values)
    if len(result) != expected_length:
        raise ValueError(f"期望长度为 {expected_length}，实际收到 {len(result)}。")
    return result


@dataclass(slots=True)
class JointMotorCommand:
    """关节命令映射后的电机命令容器。

    这个类为什么存在：
    - 映射结果不是单个数字，而是一整组电机位置、速度、力矩。
    - 用类把它们包起来，主流程更清晰，也更方便测试。

    它在主流程里承担什么角色：
    - 它是“翻译后的命令包”。
    - 前面由映射函数生成，后面交给执行记录器或真实执行层。

    属性说明：
    - `motor_positions` (`list[float]`)：
      电机目标位置列表。
      例子：`[0.4, 0.1, -0.1, 0.0, -0.2, 0.3, 0.0]`
    - `motor_velocities` (`list[float]`)：
      电机目标速度列表。
    - `motor_torques` (`list[float]`)：
      电机目标力矩列表。
    """

    motor_positions: list[float]
    motor_velocities: list[float]
    motor_torques: list[float]


@dataclass(slots=True)
class MotorJointState:
    """电机反馈回算后的关节状态容器。

    这个类为什么存在：
    - 执行层返回的通常是电机侧数据。
    - 但控制和分析更常站在“关节侧”理解系统，所以需要一个回算结果容器。

    它在主流程里承担什么角色：
    - 它是“反向翻译结果包”。
    - 前面由电机反馈回算得到，后面交给分析、对照或控制逻辑。

    属性说明：
    - `joint_positions` (`list[float]`)：
      关节位置列表。
    - `joint_velocities` (`list[float]`)：
      关节速度列表。
    - `joint_torques` (`list[float]`)：
      关节力矩列表。
    """

    joint_positions: list[float]
    joint_velocities: list[float]
    joint_torques: list[float]


def joint_to_motor_commands(
    joint_positions: Sequence[float],
    joint_velocities: Sequence[float],
    joint_torques: Sequence[float],
) -> JointMotorCommand:
    """把关节空间命令翻译成电机空间命令。

    这个函数是干什么的：
    - 输入关节位置、关节速度、关节力矩。
    - 输出执行层更容易理解的电机位置、速度、力矩。

    它在整个项目流程里处于哪一步：
    - 位于动力学接口之后、执行层之前。

    输入参数：
    - `joint_positions` (`Sequence[float]`)：
      关节位置列表，长度应为 7。
      例子：`[0.2, -0.1, 0.15, 0.0, 0.0, 0.0, 0.0]`
    - `joint_velocities` (`Sequence[float]`)：
      关节速度列表，长度应为 7。
    - `joint_torques` (`Sequence[float]`)：
      关节力矩列表，长度应为 7。

    返回值：
    - `JointMotorCommand`：
      包含 `motor_positions`、`motor_velocities`、`motor_torques` 三个列表。

    一个具体例子：
    ```python
    result = joint_to_motor_commands(
        joint_positions=[0.2, -0.1, 0.15, 0.0, 0.0, 0.0, 0.0],
        joint_velocities=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joint_torques=[1.0, 0.5, -0.2, 0.0, 0.0, 0.0, 0.0],
    )
    ```

    新手最容易误解的点：
    - 它不是“把每个关节值原样抄到同编号电机”。
    - 前 3 个通道存在减速比和耦合关系。
    """
    joint_positions = _as_command_vector(joint_positions)
    joint_velocities = _as_command_vector(joint_velocities)
    joint_torques = _as_command_vector(joint_torques)

    motor_positions = [0.0] * DEFAULT_COMMAND_DOF
    motor_velocities = [0.0] * DEFAULT_COMMAND_DOF
    motor_torques = [0.0] * DEFAULT_COMMAND_DOF

    # 前 3 个通道不是简单的一一对应。
    # 这里保留原项目中的耦合关系：
    # - 第 1 关节由第 1 电机单独负责。
    # - 第 2、3 关节会组合成第 2、3 电机的命令。
    motor_positions[0] = JOINT_1_REDUCTION_RATIO * joint_positions[0]
    motor_positions[1] = (
        JOINT_2_REDUCTION_RATIO * joint_positions[1]
        + JOINT_3_REDUCTION_RATIO * joint_positions[2]
    )
    motor_positions[2] = (
        JOINT_2_REDUCTION_RATIO * joint_positions[1]
        - JOINT_3_REDUCTION_RATIO * joint_positions[2]
    )

    # 后 4 个通道目前按学习版可确认逻辑直接或取反传递。
    motor_positions[3] = joint_positions[3]
    motor_positions[4] = -joint_positions[4]
    motor_positions[5] = -joint_positions[5]
    motor_positions[6] = -joint_positions[6]

    motor_velocities[0] = JOINT_1_REDUCTION_RATIO * joint_velocities[0]
    motor_velocities[1] = (
        JOINT_2_REDUCTION_RATIO * joint_velocities[1]
        + JOINT_3_REDUCTION_RATIO * joint_velocities[2]
    )
    motor_velocities[2] = (
        JOINT_2_REDUCTION_RATIO * joint_velocities[1]
        - JOINT_3_REDUCTION_RATIO * joint_velocities[2]
    )

    motor_torques[0] = joint_torques[0] / JOINT_1_REDUCTION_RATIO
    motor_torques[1] = (
        JOINT_3_REDUCTION_RATIO * joint_torques[1]
        + JOINT_2_REDUCTION_RATIO * joint_torques[2]
    ) / (2 * JOINT_2_REDUCTION_RATIO * JOINT_3_REDUCTION_RATIO)
    motor_torques[2] = (
        JOINT_3_REDUCTION_RATIO * joint_torques[1]
        - JOINT_2_REDUCTION_RATIO * joint_torques[2]
    ) / (2 * JOINT_2_REDUCTION_RATIO * JOINT_3_REDUCTION_RATIO)

    # 这里特意保留原项目对应逻辑的实际结果。
    # 后 4 个速度/力矩通道没有先被赋入非零值，又直接对自身取负，
    # 所以最终结果仍然是 0.0。
    # 这不是“更优设计”，而是历史行为保留，方便对照原版。
    motor_velocities[3] = -motor_velocities[3]
    motor_velocities[4] = -motor_velocities[4]
    motor_velocities[5] = -motor_velocities[5]
    motor_velocities[6] = -motor_velocities[6]

    motor_torques[3] = -motor_torques[3]
    motor_torques[4] = -motor_torques[4]
    motor_torques[5] = -motor_torques[5]
    motor_torques[6] = -motor_torques[6]

    return JointMotorCommand(
        motor_positions=motor_positions,
        motor_velocities=motor_velocities,
        motor_torques=motor_torques,
    )


def motor_to_joint_state(
    motor_positions: Sequence[float],
    motor_velocities: Sequence[float],
    motor_torques: Sequence[float],
) -> MotorJointState:
    """把电机空间状态回算成关节空间状态。

    这个函数是干什么的：
    - 把执行层或仿真层返回的电机状态，重新翻译回关节状态。

    它在整个项目流程里处于哪一步：
    - 更常出现在反馈分析阶段，而不是前向主链路阶段。

    输入参数：
    - `motor_positions` (`Sequence[float]`)：
      电机位置列表。
    - `motor_velocities` (`Sequence[float]`)：
      电机速度列表。
    - `motor_torques` (`Sequence[float]`)：
      电机力矩列表。

    返回值：
    - `MotorJointState`：
      含关节位置、速度、力矩。

    最小例子：
    ```python
    joint_state = motor_to_joint_state(
        motor_positions=[0.0] * 7,
        motor_velocities=[0.0] * 7,
        motor_torques=[0.0] * 7,
    )
    # joint_state.joint_positions 也是长度为 7 的列表
    ```
    """
    motor_positions = _as_command_vector(motor_positions)
    motor_velocities = _as_command_vector(motor_velocities)
    motor_torques = _as_command_vector(motor_torques)

    joint_positions = [0.0] * DEFAULT_COMMAND_DOF
    joint_velocities = [0.0] * DEFAULT_COMMAND_DOF
    joint_torques = [0.0] * DEFAULT_COMMAND_DOF

    # 因为前向映射里第 2、3 关节和第 2、3 电机存在耦合，
    # 所以反向回算时也必须使用对应的反推公式，而不是简单逐项复制。
    joint_positions[0] = motor_positions[0] / JOINT_1_REDUCTION_RATIO
    joint_positions[1] = (motor_positions[1] + motor_positions[2]) / (2 * JOINT_2_REDUCTION_RATIO)
    joint_positions[2] = (motor_positions[1] - motor_positions[2]) / (2 * JOINT_3_REDUCTION_RATIO)

    joint_velocities[0] = motor_velocities[0] / JOINT_1_REDUCTION_RATIO
    joint_velocities[1] = (motor_velocities[1] + motor_velocities[2]) / (2 * JOINT_2_REDUCTION_RATIO)
    joint_velocities[2] = (motor_velocities[1] - motor_velocities[2]) / (2 * JOINT_3_REDUCTION_RATIO)

    joint_torques[0] = JOINT_1_REDUCTION_RATIO * motor_torques[0]
    joint_torques[1] = JOINT_2_REDUCTION_RATIO * motor_torques[1] + JOINT_2_REDUCTION_RATIO * motor_torques[2]
    joint_torques[2] = JOINT_3_REDUCTION_RATIO * motor_torques[1] - JOINT_3_REDUCTION_RATIO * motor_torques[2]

    return MotorJointState(
        joint_positions=joint_positions,
        joint_velocities=joint_velocities,
        joint_torques=joint_torques,
    )
