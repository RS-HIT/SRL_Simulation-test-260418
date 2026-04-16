"""离线教学仿真与绘图工具。

这个文件的目标不是替代原项目真机或真仿真器，而是把学习版的过程“看得见”：
- 根据主流程输出的期望关节角，构造一个简单的教学用“实际执行”响应。
- 记录时间序列数据到 CSV。
- 生成每个关节的“期望角度 vs 模拟实际角度”曲线图和误差图。

注意：
- 这里的“实际角度”不是原版真实硬件反馈。
- 它只是一个离线教学替身，用来帮助新手观察过程。
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import matplotlib.pyplot as plt
from matplotlib import font_manager, rcParams

from ..core.controller_pipeline import OfflinePipelineStep


def _configure_matplotlib_fonts() -> None:
    """为中文图表选择一个可用字体，避免中文标题和图例报警告。"""
    available_font_names = {font.name for font in font_manager.fontManager.ttflist}
    preferred_fonts = [
        "Microsoft YaHei",
        "SimHei",
        "SimSun",
        "Microsoft JhengHei",
    ]

    for font_name in preferred_fonts:
        if font_name in available_font_names:
            rcParams["font.sans-serif"] = [font_name]
            rcParams["axes.unicode_minus"] = False
            return

    # 如果当前环境确实没有常见中文字体，就退回默认字体。
    # 这样虽然中文显示可能不完美，但至少不阻断绘图流程。
    rcParams["axes.unicode_minus"] = False


@dataclass(slots=True)
class OfflineVisualizationStep:
    """单步可视化记录。

    属性：
    - `time_seconds`：当前时间，单位秒。
    - `desired_joint_position`：期望关节角。
    - `actual_joint_position`：教学仿真得到的模拟实际关节角。
    - `joint_error`：实际角度减期望角度。
    """

    time_seconds: float
    desired_joint_position: list[float]
    actual_joint_position: list[float]
    joint_error: list[float]


class OfflineJointResponseSimulator:
    """教学版关节响应模拟器。

    大白话：
    - 它假装“执行层不会瞬间到位，而是会慢慢追上目标”。

    正式一点的说法：
    - 它用一个简单的一阶跟踪模型，把期望关节角转换成模拟实际关节角。

    注意：
    - 这不是物理精确仿真。
    - 它只用于把过程画出来，帮助理解“期望”和“实际”为什么常常不一样。
    """

    def __init__(self, response_gain: float) -> None:
        if not 0.0 < response_gain <= 1.0:
            raise ValueError("response_gain 必须在 (0, 1] 范围内。")
        self.response_gain = float(response_gain)

    def simulate(
        self,
        pipeline_steps: Iterable[OfflinePipelineStep],
        joint_dof: int,
        control_period: float,
    ) -> list[OfflineVisualizationStep]:
        """根据主链路结果生成教学版模拟实际关节轨迹。"""
        actual_joint_position = [0.0] * joint_dof
        visualization_steps: list[OfflineVisualizationStep] = []

        for step in pipeline_steps:
            desired_joint_position = step.desired_joint_position[:joint_dof]

            # 这里故意不让“实际值”一步到位，而是只朝目标前进一部分。
            # 这样图里就能看到一个典型的跟踪过程：实际值逐步逼近期望值。
            for joint_index in range(joint_dof):
                actual_joint_position[joint_index] = actual_joint_position[joint_index] + (
                    self.response_gain * (desired_joint_position[joint_index] - actual_joint_position[joint_index])
                )

            joint_error = [
                actual_joint_position[joint_index] - desired_joint_position[joint_index]
                for joint_index in range(joint_dof)
            ]
            visualization_steps.append(
                OfflineVisualizationStep(
                    time_seconds=step.step_index * control_period,
                    desired_joint_position=desired_joint_position.copy(),
                    actual_joint_position=actual_joint_position.copy(),
                    joint_error=joint_error,
                )
            )

        return visualization_steps


def save_visualization_csv(
    output_csv_path: Path,
    visualization_steps: list[OfflineVisualizationStep],
    joint_dof: int,
) -> None:
    """把可视化时序数据保存成 CSV。"""
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)

    headers = ["time_seconds"]
    for joint_index in range(joint_dof):
        headers.append(f"joint_{joint_index + 1}_desired")
        headers.append(f"joint_{joint_index + 1}_actual")
        headers.append(f"joint_{joint_index + 1}_error")

    with output_csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(headers)
        for step in visualization_steps:
            row = [step.time_seconds]
            for joint_index in range(joint_dof):
                row.append(step.desired_joint_position[joint_index])
                row.append(step.actual_joint_position[joint_index])
                row.append(step.joint_error[joint_index])
            writer.writerow(row)


def save_joint_tracking_plot(
    output_plot_path: Path,
    visualization_steps: list[OfflineVisualizationStep],
    joint_dof: int,
    title_prefix: str,
) -> None:
    """生成“期望角度 vs 模拟实际角度”图。"""
    _configure_matplotlib_fonts()
    output_plot_path.parent.mkdir(parents=True, exist_ok=True)

    times = [step.time_seconds for step in visualization_steps]
    figure, axes = plt.subplots(joint_dof, 1, figsize=(12, max(8, 2.4 * joint_dof)), sharex=True)
    if joint_dof == 1:
        axes = [axes]

    for joint_index, axis in enumerate(axes):
        desired_values = [step.desired_joint_position[joint_index] for step in visualization_steps]
        actual_values = [step.actual_joint_position[joint_index] for step in visualization_steps]
        axis.plot(times, desired_values, label="期望角度", linewidth=2.0)
        axis.plot(times, actual_values, label="模拟实际角度", linewidth=1.8, linestyle="--")
        axis.set_ylabel(f"关节{joint_index + 1}\n角度(rad)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    axes[-1].set_xlabel("时间 (s)")
    figure.suptitle(f"{title_prefix} - 关节角跟踪图", fontsize=14)
    figure.tight_layout()
    figure.savefig(output_plot_path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def save_joint_error_plot(
    output_plot_path: Path,
    visualization_steps: list[OfflineVisualizationStep],
    joint_dof: int,
    title_prefix: str,
) -> None:
    """生成关节误差图。"""
    _configure_matplotlib_fonts()
    output_plot_path.parent.mkdir(parents=True, exist_ok=True)

    times = [step.time_seconds for step in visualization_steps]
    figure, axes = plt.subplots(joint_dof, 1, figsize=(12, max(8, 2.2 * joint_dof)), sharex=True)
    if joint_dof == 1:
        axes = [axes]

    for joint_index, axis in enumerate(axes):
        error_values = [step.joint_error[joint_index] for step in visualization_steps]
        axis.plot(times, error_values, label="实际 - 期望", linewidth=1.8, color="#c44e52")
        axis.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
        axis.set_ylabel(f"关节{joint_index + 1}\n误差(rad)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    axes[-1].set_xlabel("时间 (s)")
    figure.suptitle(f"{title_prefix} - 关节误差图", fontsize=14)
    figure.tight_layout()
    figure.savefig(output_plot_path, dpi=160, bbox_inches="tight")
    plt.close(figure)
