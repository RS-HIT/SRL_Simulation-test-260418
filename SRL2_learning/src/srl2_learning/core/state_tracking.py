"""把离散状态补成更完整的状态信息。

这个文件在整条链路里的位置：
- 它位于“规划器给出位置”之后、“动力学和映射开始使用状态”之前。

这个文件解决什么问题：
- 规划器或反馈层经常只直接给出位置。
- 但后面的模块通常还需要速度和加速度。
- 这里负责把“这一拍和上一拍”的关系整理成更完整的状态。

如果没有它，项目会卡在哪：
- 我们会停在“我知道现在的位置”，但还不知道“变化有多快、变化趋势如何”。

当前是否为教学替身：
- 不完全是占位壳。
- 关键差分公式和低通滤波公式保留了原项目中可明确确认的思路。
- 但这里是 Python 教学实现，不是原始 C++/Eigen 本体。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence


def _vector(values: Sequence[float]) -> list[float]:
    """把输入统一整理成 `list[float]`。

    参数：
    - `values` (`Sequence[float]`)：任意可遍历数字序列。

    返回值：
    - `list[float]`：逐项转成浮点数后的列表。

    例子：
    ```python
    _vector((1, 2, 3))
    # 返回 [1.0, 2.0, 3.0]
    ```
    """
    return [float(value) for value in values]


def _vector_subtract(left: Sequence[float], right: Sequence[float]) -> list[float]:
    """向量逐项相减。

    参数：
    - `left` (`Sequence[float]`)：左侧向量。
    - `right` (`Sequence[float]`)：右侧向量。

    返回值：
    - `list[float]`：`left[i] - right[i]` 组成的新列表。

    关于 `zip(left, right, strict=True)`：
    - 大白话：它会把两个列表按相同下标一一配对。
    - 例子：`[10, 20]` 和 `[1, 2]` 会配成 `(10, 1)`、`(20, 2)`。
    - `strict=True` 的意思是：如果两个列表长度不一样，就立刻报错，而不是悄悄少算一部分。

    例子：
    ```python
    _vector_subtract([3, 5], [1, 2])
    # 返回 [2.0, 3.0]
    ```
    """
    return [float(a) - float(b) for a, b in zip(left, right, strict=True)]


def _vector_scale(values: Sequence[float], scale: float) -> list[float]:
    """把向量整体乘以同一个系数。

    参数：
    - `values` (`Sequence[float]`)：原始向量。
    - `scale` (`float`)：缩放系数。

    返回值：
    - `list[float]`：每一项都乘以 `scale` 后的新列表。

    例子：
    ```python
    _vector_scale([1, 2, 3], 0.5)
    # 返回 [0.5, 1.0, 1.5]
    ```
    """
    return [float(value) * scale for value in values]


def _copy_desired_motion_state(state: "DesiredMotionState") -> "DesiredMotionState":
    """返回期望状态快照，避免外部直接改内部状态。"""
    return DesiredMotionState(
        position=state.position.copy(),
        velocity=state.velocity.copy(),
        acceleration=state.acceleration.copy(),
        position_last=state.position_last.copy(),
        velocity_last=state.velocity_last.copy(),
    )


def _copy_motion_observation_state(state: "MotionObservationState") -> "MotionObservationState":
    """返回观测状态快照。"""
    return MotionObservationState(
        position=state.position.copy(),
        velocity_from_position=state.velocity_from_position.copy(),
        acceleration_from_position=state.acceleration_from_position.copy(),
        position_last=state.position_last.copy(),
        velocity_from_position_last=state.velocity_from_position_last.copy(),
    )


def _copy_differentiated_signal_state(state: "DifferentiatedSignalState") -> "DifferentiatedSignalState":
    """返回通用差分状态快照。"""
    return DifferentiatedSignalState(
        position=state.position.copy(),
        velocity=state.velocity.copy(),
        acceleration=state.acceleration.copy(),
        position_last=state.position_last.copy(),
        velocity_last=state.velocity_last.copy(),
    )


@dataclass(slots=True)
class DesiredMotionState:
    """期望状态容器。

    这个类是干什么的：
    - 把“期望位置、期望速度、期望加速度”放在一起保存。

    它在整个项目流程里处于哪一步：
    - 规划器输出位置之后，会进入这个容器。
    - 后面的动力学接口和映射层会继续使用它。

    属性说明：
    - `position` (`list[float]`)：
      当前这一拍的期望位置。
      例子：`[0.1, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0]`
    - `velocity` (`list[float]`)：
      由差分计算得到的期望速度。
    - `acceleration` (`list[float]`)：
      由差分计算得到的期望加速度。
    - `position_last` (`list[float]`)：
      上一拍的位置。
      为什么需要它：没有上一拍，就没法算“这一拍比上一拍快了多少”。
    - `velocity_last` (`list[float]`)：
      上一拍的速度。
      为什么需要它：没有上一拍速度，就没法算当前加速度。
    """

    position: list[float]
    velocity: list[float]
    acceleration: list[float]
    position_last: list[float]
    velocity_last: list[float]

    @classmethod
    def create(cls, dof: int) -> "DesiredMotionState":
        """创建一个指定维度的全零期望状态。

        参数：
        - `dof` (`int`)：向量长度。

        返回值：
        - `DesiredMotionState`：所有列表都初始化为 0.0。
        """
        zeros = [0.0] * dof
        return cls(
            position=zeros.copy(),
            velocity=zeros.copy(),
            acceleration=zeros.copy(),
            position_last=zeros.copy(),
            velocity_last=zeros.copy(),
        )


@dataclass(slots=True)
class MotionObservationState:
    """观测状态容器。

    这个类是干什么的：
    - 保存“实际观察到”的位置、速度、加速度。

    属性说明：
    - `position` (`list[float]`)：当前观测位置。
    - `velocity_from_position` (`list[float]`)：
      由位置差分算出的速度。
    - `acceleration_from_position` (`list[float]`)：
      由速度差分算出的加速度。
    - `position_last` (`list[float]`)：
      上一拍观测位置。
    - `velocity_from_position_last` (`list[float]`)：
      上一拍由位置算出的速度。
    """

    position: list[float]
    velocity_from_position: list[float]
    acceleration_from_position: list[float]
    position_last: list[float]
    velocity_from_position_last: list[float]

    @classmethod
    def create(cls, dof: int) -> "MotionObservationState":
        """创建一个指定维度的全零观测状态。"""
        zeros = [0.0] * dof
        return cls(
            position=zeros.copy(),
            velocity_from_position=zeros.copy(),
            acceleration_from_position=zeros.copy(),
            position_last=zeros.copy(),
            velocity_from_position_last=zeros.copy(),
        )


@dataclass(slots=True)
class DifferentiatedSignalState:
    """通用差分信号容器。

    这个类是干什么的：
    - 有些信号不一定是完整机械臂状态，但依然需要做速度和加速度估计。

    属性说明：
    - `position` (`list[float]`)：当前输入信号值。
    - `velocity` (`list[float]`)：差分速度。
    - `acceleration` (`list[float]`)：差分加速度。
    - `position_last` (`list[float]`)：上一拍输入值。
    - `velocity_last` (`list[float]`)：上一拍差分速度。
    """

    position: list[float]
    velocity: list[float]
    acceleration: list[float]
    position_last: list[float]
    velocity_last: list[float]

    @classmethod
    def create(cls, dof: int) -> "DifferentiatedSignalState":
        """创建一个指定维度的全零通用差分状态。"""
        zeros = [0.0] * dof
        return cls(
            position=zeros.copy(),
            velocity=zeros.copy(),
            acceleration=zeros.copy(),
            position_last=zeros.copy(),
            velocity_last=zeros.copy(),
        )


@dataclass(slots=True)
class LowPassFilterState:
    """低通滤波器状态。

    这个类是干什么的：
    - 低通滤波不是只看当前值，还要记住上一拍的滤波结果。

    属性说明：
    - `value` (`list[float]`)：当前滤波输出。
    - `value_last` (`list[float]`)：上一拍滤波输出。
      为什么需要它：低通滤波本质上是在“旧结果”和“新输入”之间做折中。
    """

    value: list[float]
    value_last: list[float]

    @classmethod
    def create(cls, dof: int) -> "LowPassFilterState":
        """创建一个指定维度的全零滤波状态。"""
        zeros = [0.0] * dof
        return cls(value=zeros.copy(), value_last=zeros.copy())


class DesiredMotionTracker:
    """把规划位置补成完整期望状态的跟踪器。

    这个类为什么存在：
    - 上游规划器可能只给出位置。
    - 下游模块常常还需要速度和加速度。

    它在主流程中承担什么角色：
    - 它是“期望状态加工者”。
    - 把规划器的一维输出，补成更完整的状态包。

    属性说明：
    - `control_period` (`float`)：
      控制周期，单位秒。
      例子：`0.005` 表示每 5 毫秒更新一次。
    - `state` (`DesiredMotionState | None`)：
      内部保存的最新状态。
    """

    def __init__(self, control_period: float) -> None:
        self.control_period = control_period
        self.state: DesiredMotionState | None = None

    def reset(self, dof: int) -> None:
        """开始一段新运动前重置内部状态。

        参数：
        - `dof` (`int`)：状态向量长度。

        返回值：
        - 无。

        例子：
        ```python
        tracker = DesiredMotionTracker(0.005)
        tracker.reset(7)
        ```
        """
        self.state = DesiredMotionState.create(dof)

    def update(self, position: Sequence[float], first_time: bool = False) -> DesiredMotionState:
        """根据当前位置更新期望位置、速度和加速度。

        参数：
        - `position` (`Sequence[float]`)：
          当前这一拍的期望位置。
        - `first_time` (`bool`)：
          是否是本段运动的第一拍。
          第一拍没有上一拍可参考，所以通常不做差分。

        返回值：
        - `DesiredMotionState`：
          当前拍的完整期望状态快照。

        具体例子：
        ```python
        tracker = DesiredMotionTracker(0.1)
        tracker.reset(2)
        tracker.update([0.0, 0.0], first_time=True)
        state = tracker.update([0.1, 0.2], first_time=False)
        # state.velocity 约为 [1.0, 2.0]
        ```
        """
        if self.state is None:
            self.reset(len(position))

        assert self.state is not None
        current_position = _vector(position)
        self.state.position = current_position

        # 为什么要保存上一拍值：
        # - 速度 = 位置变化量 / 时间
        # - 加速度 = 速度变化量 / 时间
        # 如果没有上一拍，就没法做这两个差分。
        if not first_time:
            self.state.velocity = _vector_scale(
                _vector_subtract(self.state.position, self.state.position_last),
                1.0 / self.control_period,
            )
            self.state.acceleration = _vector_scale(
                _vector_subtract(self.state.velocity, self.state.velocity_last),
                1.0 / self.control_period,
            )

        self.state.position_last = self.state.position.copy()
        self.state.velocity_last = self.state.velocity.copy()
        return _copy_desired_motion_state(self.state)


class MotionObservationTracker:
    """把反馈位置补成观测速度和加速度的跟踪器。

    属性说明：
    - `control_period` (`float`)：控制周期。
    - `state` (`MotionObservationState | None`)：内部观测状态缓存。
    """

    def __init__(self, control_period: float) -> None:
        self.control_period = control_period
        self.state: MotionObservationState | None = None

    def reset(self, dof: int) -> None:
        """重置观测状态。"""
        self.state = MotionObservationState.create(dof)

    def update(self, position: Sequence[float], first_time: bool = False) -> MotionObservationState:
        """根据反馈位置更新观测状态。

        参数：
        - `position` (`Sequence[float]`)：当前反馈位置。
        - `first_time` (`bool`)：是否为第一拍。

        返回值：
        - `MotionObservationState`：观测状态快照。
        """
        if self.state is None:
            self.reset(len(position))

        assert self.state is not None
        self.state.position = _vector(position)

        if not first_time:
            self.state.velocity_from_position = _vector_scale(
                _vector_subtract(self.state.position, self.state.position_last),
                1.0 / self.control_period,
            )
            self.state.acceleration_from_position = _vector_scale(
                _vector_subtract(
                    self.state.velocity_from_position,
                    self.state.velocity_from_position_last,
                ),
                1.0 / self.control_period,
            )

        self.state.position_last = self.state.position.copy()
        self.state.velocity_from_position_last = self.state.velocity_from_position.copy()
        return _copy_motion_observation_state(self.state)


class DifferentiatedSignalTracker:
    """通用差分工具。

    属性说明：
    - `control_period` (`float`)：控制周期。
    - `state` (`DifferentiatedSignalState | None`)：内部差分状态缓存。
    """

    def __init__(self, control_period: float) -> None:
        self.control_period = control_period
        self.state: DifferentiatedSignalState | None = None

    def reset(self, dof: int) -> None:
        """重置通用差分状态。"""
        self.state = DifferentiatedSignalState.create(dof)

    def update(
        self,
        position: Sequence[float],
        first_time: bool = False,
        step_scale: float = 1.0,
    ) -> DifferentiatedSignalState:
        """更新任意离散信号的差分结果。

        参数：
        - `position` (`Sequence[float]`)：当前输入信号。
        - `first_time` (`bool`)：是否是第一拍。
        - `step_scale` (`float`)：
          有效步长缩放因子。
          例子：如果某些输入每次相当于跨了 2 个小步，可以用它做修正。

        返回值：
        - `DifferentiatedSignalState`：当前差分结果快照。

        新手容易误解的点：
        - 这里不是对所有维度都自动做复杂处理。
        - 按原项目当前可确认逻辑，只对前 3 个量做主要差分处理。
        """
        if self.state is None:
            self.reset(len(position))

        assert self.state is not None
        self.state.position = _vector(position)

        # 为什么这里只处理前 3 个量：
        # - 这是为了保留原项目当前这段逻辑的主要使用范围。
        # - 学习版先忠实保留可确认行为，而不是擅自把规则扩展到所有维度。
        for index in range(min(3, len(self.state.position))):
            if not first_time and self.state.position[index] != self.state.position_last[index]:
                self.state.velocity[index] = (
                    (self.state.position[index] - self.state.position_last[index])
                    / self.control_period
                    / step_scale
                )
                self.state.acceleration[index] = (
                    (self.state.velocity[index] - self.state.velocity_last[index])
                    / self.control_period
                    / step_scale
                )

        self.state.position_last = self.state.position.copy()
        self.state.velocity_last = self.state.velocity.copy()
        return _copy_differentiated_signal_state(self.state)


class LowPassFilter:
    """一阶低通滤波器。

    这个类为什么存在：
    - 真实测量值经常会抖。
    - 直接把抖动信号送给后续模块，结果也会跟着抖。

    它在主流程中承担什么角色：
    - 它是“平滑器”。
    - 让后续逻辑看到的是更平稳的趋势，而不是生硬的原始波动。

    属性说明：
    - `alpha` (`float`)：
      当前输入占多大权重。
      例子：`alpha=0.2` 表示新输入占 20%，上一拍结果占 80%。
      为什么需要它：它决定“跟得快一点”还是“更平滑一点”。
    - `state` (`LowPassFilterState | None`)：
      内部滤波状态缓存。
    """

    def __init__(self, alpha: float = 0.2) -> None:
        self.alpha = alpha
        self.state: LowPassFilterState | None = None

    def reset(self, dof: int) -> None:
        """重置滤波状态。"""
        self.state = LowPassFilterState.create(dof)

    def update(self, value: Sequence[float]) -> list[float]:
        """根据新输入更新滤波结果。

        参数：
        - `value` (`Sequence[float]`)：
          当前原始输入值。

        返回值：
        - `list[float]`：
          当前滤波后的结果。

        例子：
        ```python
        filt = LowPassFilter(alpha=0.5)
        filt.reset(2)
        filt.update([10, 0])   # 大约得到 [5.0, 0.0]
        filt.update([10, 0])   # 大约得到 [7.5, 0.0]
        ```
        """
        if self.state is None:
            self.reset(len(value))

        assert self.state is not None
        self.state.value = _vector(value)

        # 这里用到了 zip(self.state.value_last, self.state.value, strict=True)。
        # 大白话：
        # - 它会把“上一拍输出”和“当前输入”按同一位置一一配对。
        # - 例如 [1, 2] 和 [10, 20] 会配成 (1, 10)、(2, 20)。
        # 下面这行的公式是：
        # - 新输出 = (1 - alpha) * 上一拍输出 + alpha * 当前输入
        # 所以 alpha 越大，越跟随当前输入；alpha 越小，越平滑。
        self.state.value = [
            (1.0 - self.alpha) * last_value + self.alpha * current_value
            for last_value, current_value in zip(
                self.state.value_last,
                self.state.value,
                strict=True,
            )
        ]
        self.state.value_last = self.state.value.copy()
        return self.state.value
