"""离线最小链路示例。

运行效果：
- 执行学习版最小主链路。
- 生成一份教学用“模拟实际关节角”时序。
- 在 `logs/` 下保存 CSV 和两张 PNG 图。

运行方式：
- 在 `SRL2_learning` 目录下：
  `python examples/minimal_joint_motion.py`
- 在仓库根目录下：
  `python SRL2_learning/scripts/run_minimal_example.py`
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.offline_joint_visualization import (
    OfflineJointResponseSimulator,
    save_joint_error_plot,
    save_joint_tracking_plot,
    save_visualization_csv,
)
from srl2_learning.core.controller_pipeline import LearningRobotController


def main() -> None:
    """运行最小主链路，并输出可视化结果。"""
    config_path = PROJECT_ROOT / "configs" / "offline_demo_config.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))

    controller = LearningRobotController(
        control_period=config["control_period_seconds"],
        joint_dof=config["joint_dof"],
        command_dof=config["command_dof"],
    )

    model_path = (config_path.parent / config["model_reference_path"]).resolve()
    controller.load_reference_model(
        model_path=str(model_path),
        end_link_name=config["end_link_name"],
        tool_offset=config["tool_offset"],
    )

    step_results = controller.run_joint_space_motion(
        start_joint_command=config["start_joint_command"],
        target_joint_command=config["target_joint_command"],
        max_velocity=config["max_velocity"],
        max_acceleration=config["max_acceleration"],
    )

    simulator = OfflineJointResponseSimulator(
        response_gain=config["simulation_response_gain"],
    )
    visualization_steps = simulator.simulate(
        pipeline_steps=step_results,
        joint_dof=config["joint_dof"],
        control_period=config["control_period_seconds"],
    )

    output_directory = PROJECT_ROOT / "logs" / config["output_directory_name"]
    csv_path = output_directory / "joint_tracking.csv"
    tracking_plot_path = output_directory / "joint_tracking.png"
    error_plot_path = output_directory / "joint_error.png"

    save_visualization_csv(
        output_csv_path=csv_path,
        visualization_steps=visualization_steps,
        joint_dof=config["joint_dof"],
    )
    save_joint_tracking_plot(
        output_plot_path=tracking_plot_path,
        visualization_steps=visualization_steps,
        joint_dof=config["joint_dof"],
        title_prefix=config["plot_title_prefix"],
    )
    save_joint_error_plot(
        output_plot_path=error_plot_path,
        visualization_steps=visualization_steps,
        joint_dof=config["joint_dof"],
        title_prefix=config["plot_title_prefix"],
    )

    first_step = step_results[0]
    last_step = step_results[-1]
    last_visualization_step = visualization_steps[-1]

    print("离线最小链路运行完成。")
    print(f"总步数: {len(step_results)}")
    print(f"首步期望关节位置: {first_step.desired_joint_position}")
    print(f"末步期望关节位置: {last_step.desired_joint_position}")
    print(f"末步模拟实际关节位置: {last_visualization_step.actual_joint_position}")
    print(f"末步电机位置: {last_step.mapped_motor_command.motor_positions}")
    print(f"动力学说明: {last_step.dynamics_result.note}")
    print(f"CSV 日志: {csv_path}")
    print(f"关节跟踪图: {tracking_plot_path}")
    print(f"关节误差图: {error_plot_path}")


if __name__ == "__main__":
    main()
