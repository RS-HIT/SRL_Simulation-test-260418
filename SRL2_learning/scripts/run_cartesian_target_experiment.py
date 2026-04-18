"""运行四坐标系版本的末端坐标驱动实验。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.cartesian_visualization import (
    export_cartesian_case_plots,
    export_cartesian_plots,
    export_cartesian_summary_csv,
    export_cartesian_tracking_csv,
)
from srl2_learning.adapters.frame_visualizer import FrameVisualizationConfig, replay_cartesian_experiment_in_viewer
from srl2_learning.adapters.frame_visualizer import ViewerCameraConfig
from srl2_learning.experiments.cartesian_target_experiment import (
    run_multi_target_experiment,
    run_single_target_experiment,
    run_tcp_sensitivity_experiment,
)
from srl2_learning.kinematics.model_context import load_real_mesh_model_context
from srl2_learning.kinematics.tool_frame import ToolFrameConfig, load_tool_frame_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="运行四坐标系版本的末端实验。默认打开 viewer，并显示 world/base/flange/tool。"
    )
    parser.add_argument("--config", type=Path, default=PROJECT_ROOT / "configs" / "cartesian_target_experiment.json", help="实验配置文件路径。")
    parser.add_argument("--target-x", type=float, help="覆盖目标点 x 坐标。")
    parser.add_argument("--target-y", type=float, help="覆盖目标点 y 坐标。")
    parser.add_argument("--target-z", type=float, help="覆盖目标点 z 坐标。")
    parser.add_argument("--tool-config", type=Path, help="工具坐标系配置文件路径。")
    parser.add_argument("--viewer", action="store_true", help="强制开启 viewer。")
    parser.add_argument("--no-viewer", action="store_true", help="显式关闭 viewer。")
    parser.add_argument("--export-dir", type=Path, help="导出目录。")
    parser.add_argument("--ik-max-iter", type=int, help="IK 最大迭代次数。")
    parser.add_argument("--ik-tol", type=float, help="IK 收敛阈值。")
    parser.add_argument("--experiment-mode", choices=["single", "multi", "tcp_sensitivity"], help="实验模式。")
    parser.add_argument("--use-tool-frame", action="store_true", help="强制启用工具坐标系偏移。")
    parser.add_argument("--no-tool-frame", action="store_true", help="强制关闭工具坐标系偏移。")
    return parser.parse_args()


def _load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _resolve_config_path(base_path: Path, relative_or_absolute: str | Path) -> Path:
    candidate = Path(relative_or_absolute)
    if candidate.is_absolute():
        return candidate
    return (base_path.parent / candidate).resolve()


def _load_tool_cases(cases_path: Path) -> list[ToolFrameConfig]:
    data = _load_json(cases_path)
    cases: list[ToolFrameConfig] = []
    for case in data.get("cases", []):
        cases.append(
            ToolFrameConfig(
                enabled=bool(case.get("enabled", True)),
                label=str(case.get("label", "tool_case")),
                tool_translation_xyz=[float(value) for value in case.get("tool_translation_xyz", case.get("translation_xyz", [0.0, 0.0, 0.0]))],
                tool_rotation_rpy=[float(value) for value in case.get("tool_rotation_rpy", case.get("rotation_rpy", [0.0, 0.0, 0.0]))],
            )
        )
    return cases


def _build_frame_visualization_config(config: dict) -> FrameVisualizationConfig:
    return FrameVisualizationConfig(
        show_world_frame=bool(config.get("show_world_frame", True)),
        show_base_frame=bool(config.get("show_base_frame", True)),
        show_flange_frame=bool(config.get("show_flange_frame", True)),
        show_tool_frame=bool(config.get("show_tool_frame", True)),
        show_target_point=bool(config.get("show_target_point", True)),
        show_frame_names=bool(config.get("show_frame_names", True)),
        frame_axis_length=float(config.get("frame_axis_length", 0.12)),
        frame_axis_radius=float(config.get("frame_axis_radius", 0.008)),
        frame_origin_radius=float(config.get("frame_origin_radius", 0.014)),
        target_marker_radius=float(config.get("target_marker_radius", 0.018)),
        world_frame_origin=[float(value) for value in config.get("world_frame_origin", [0.0, 0.0, 0.0])],
        print_frame_pose_each_step=bool(config.get("print_frame_pose_each_step", False)),
        print_frame_pose_every_n_steps=int(config.get("print_frame_pose_every_n_steps", 10)),
    )


def _build_viewer_camera_config(config: dict) -> ViewerCameraConfig:
    return ViewerCameraConfig(
        lookat=[float(value) for value in config.get("viewer_camera_lookat", [0.0, 0.0, 0.85])],
        distance=float(config.get("viewer_camera_distance", 2.6)),
        azimuth=float(config.get("viewer_camera_azimuth", 135.0)),
        elevation=float(config.get("viewer_camera_elevation", -18.0)),
    )


def _print_coordinate_header(
    model_context,
    tool_frame: ToolFrameConfig | None,
    use_tool_frame: bool,
    target_position: list[float],
    real_mesh_config_path: Path,
    tool_config_path: Path,
) -> None:
    reference_info = model_context.describe_reference()
    print("当前四坐标系定义：")
    print(f"- world frame 原点: {reference_info['world_frame_origin']}")
    print("- base frame 来源: runtime_urdf.world_to_base")
    print(f"- base frame 平移常量在: {real_mesh_config_path}")
    print(f"- base_position: {reference_info['base_position']}")
    print(f"- base_euler: {reference_info['base_euler']}")
    print(f"- flange frame 来源: {reference_info['reference_kind']}:{reference_info['reference_name']}")
    print(f"- 工具坐标系偏移常量在: {tool_config_path}")
    print(f"- use_tool_frame: {use_tool_frame and tool_frame is not None and tool_frame.enabled}")
    if tool_frame is not None:
        print(f"- tool_label: {tool_frame.label}")
        print(f"- tool_translation_xyz: {tool_frame.tool_translation_xyz}")
        print(f"- tool_rotation_rpy: {tool_frame.tool_rotation_rpy}")
    print("- 当前生效的关节限位（弧度）:")
    for joint_name, limits in reference_info["joint_limits_radians"].items():
        print(f"  - {joint_name}: [{limits[0]}, {limits[1]}]")
    print(f"- target_position_world: {target_position}")
    print("- 当前真正参与误差比较的是 tool frame 原点。")


def _print_result_summary(title: str, summaries) -> None:
    print(title)
    for summary in summaries:
        print(
            f"- {summary.experiment_case_id}: "
            f"base={summary.final_base_position}, "
            f"flange={summary.final_flange_position}, "
            f"tool={summary.final_tool_position}, "
            f"tool_error_norm={summary.final_tool_error_norm:.6f}, "
            f"converged={summary.converged}, "
            f"tool_label={summary.tool_label}"
        )


def main() -> None:
    args = parse_args()
    config_path = args.config.resolve()
    config = _load_json(config_path)

    experiment_mode = args.experiment_mode or config["experiment_mode"]
    if args.ik_max_iter is not None:
        config["ik_max_iterations"] = int(args.ik_max_iter)
    if args.ik_tol is not None:
        config["ik_tolerance"] = float(args.ik_tol)
    if args.viewer:
        config["viewer_enabled"] = True
    if args.no_viewer:
        config["viewer_enabled"] = False
    if args.use_tool_frame:
        config["use_tool_frame"] = True
    if args.no_tool_frame:
        config["use_tool_frame"] = False

    target_position = list(config["target_position"])
    if args.target_x is not None:
        target_position[0] = float(args.target_x)
    if args.target_y is not None:
        target_position[1] = float(args.target_y)
    if args.target_z is not None:
        target_position[2] = float(args.target_z)

    tool_config_path = _resolve_config_path(config_path, args.tool_config if args.tool_config is not None else config["tool_frame_config_path"])
    tcp_cases_path = _resolve_config_path(config_path, config["tcp_sensitivity_cases_path"])
    real_mesh_config_path = _resolve_config_path(config_path, config["real_mesh_config_path"])
    export_directory = args.export_dir.resolve() if args.export_dir is not None else (PROJECT_ROOT / "logs" / config["export_directory_name"]).resolve()

    tool_frame = load_tool_frame_config(tool_config_path)
    model_context = load_real_mesh_model_context(
        real_mesh_config_path=real_mesh_config_path,
        reference_body_name=config.get("flange_reference_body", config["reference_body_name"]),
        reference_site_name=config.get("flange_reference_site", config["reference_site_name"]),
        joint_limit_overrides_radians=config.get("joint_position_limits_radians"),
        joint_limit_overrides_degrees=config.get("joint_position_limits_degrees"),
    )
    frame_visualization_config = _build_frame_visualization_config(config)
    viewer_camera_config = _build_viewer_camera_config(config)

    _print_coordinate_header(model_context, tool_frame, bool(config["use_tool_frame"]), target_position, real_mesh_config_path, tool_config_path)

    common_kwargs = dict(
        model_context=model_context,
        initial_joint_positions=config["initial_joint_positions"],
        control_period=config["control_period_seconds"],
        max_velocity=config["max_velocity"],
        max_acceleration=config["max_acceleration"],
        ik_max_iterations=config["ik_max_iterations"],
        ik_tolerance=config["ik_tolerance"],
        ik_step_size=config["ik_step_size"],
        ik_damping=config["ik_damping"],
        hold_steps=config["hold_steps"],
    )
    export_directory.mkdir(parents=True, exist_ok=True)

    if experiment_mode == "single":
        result = run_single_target_experiment(
            target_position=target_position,
            tool_frame=tool_frame,
            use_tool_frame=config["use_tool_frame"],
            **common_kwargs,
        )
        export_cartesian_tracking_csv(export_directory / "cartesian_tracking.csv", result.samples)
        export_cartesian_summary_csv(export_directory / "cartesian_target_summary.csv", result.summaries)
        export_cartesian_plots(export_directory, result.samples, title_prefix=config["plot_title_prefix"])
        _print_result_summary("单目标实验结果：", result.summaries)
        if config["viewer_enabled"]:
            replay_cartesian_experiment_in_viewer(
                model_context=model_context,
                joint_trajectory=result.joint_trajectory,
                control_period=config["control_period_seconds"],
                playback_speed=float(config.get("viewer_playback_speed", 0.5)),
                frame_config=frame_visualization_config,
                camera_config=viewer_camera_config,
                tool_frame=tool_frame if config["use_tool_frame"] else None,
                target_position=target_position,
                target_positions=result.target_positions,
            )
        return

    if experiment_mode == "multi":
        result = run_multi_target_experiment(
            target_points=config["target_points"],
            tool_frame=tool_frame,
            use_tool_frame=config["use_tool_frame"],
            **common_kwargs,
        )
        export_cartesian_tracking_csv(export_directory / "cartesian_tracking.csv", result.samples)
        export_cartesian_summary_csv(export_directory / "cartesian_target_summary.csv", result.summaries)
        export_cartesian_plots(export_directory, result.samples, title_prefix=config["plot_title_prefix"])
        _print_result_summary("多目标实验结果：", result.summaries)
        if config["viewer_enabled"]:
            replay_cartesian_experiment_in_viewer(
                model_context=model_context,
                joint_trajectory=result.joint_trajectory,
                control_period=config["control_period_seconds"],
                playback_speed=float(config.get("viewer_playback_speed", 0.5)),
                frame_config=frame_visualization_config,
                camera_config=viewer_camera_config,
                tool_frame=tool_frame if config["use_tool_frame"] else None,
                target_position=result.target_positions[-1],
                target_positions=result.target_positions,
            )
        return

    if experiment_mode == "tcp_sensitivity":
        tool_cases = _load_tool_cases(tcp_cases_path)
        results = run_tcp_sensitivity_experiment(
            target_position=target_position,
            tcp_cases=tool_cases,
            **common_kwargs,
        )
        combined_summaries = []
        for result in results:
            case_output_directory = export_directory / result.summaries[0].tool_label
            export_cartesian_tracking_csv(case_output_directory / "cartesian_tracking.csv", result.samples)
            export_cartesian_summary_csv(case_output_directory / "cartesian_target_summary.csv", result.summaries)
            export_cartesian_plots(case_output_directory, result.samples, title_prefix=f"{config['plot_title_prefix']} - {result.summaries[0].tool_label}")
            combined_summaries.extend(result.summaries)
        export_cartesian_summary_csv(export_directory / "cartesian_target_summary.csv", combined_summaries)
        export_cartesian_case_plots(export_directory / "by_case", [sample for result in results for sample in result.samples], title_prefix=config["plot_title_prefix"])
        _print_result_summary("工具偏移敏感性实验结果：", combined_summaries)
        if config["viewer_enabled"] and results:
            replay_cartesian_experiment_in_viewer(
                model_context=model_context,
                joint_trajectory=results[0].joint_trajectory,
                control_period=config["control_period_seconds"],
                playback_speed=float(config.get("viewer_playback_speed", 0.5)),
                frame_config=frame_visualization_config,
                camera_config=viewer_camera_config,
                tool_frame=tool_cases[0],
                target_position=target_position,
                target_positions=[target_position],
            )
        return

    raise ValueError(f"不支持的实验模式: {experiment_mode}")


if __name__ == "__main__":
    main()
