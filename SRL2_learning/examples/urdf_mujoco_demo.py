"""使用外部 URDF 生成 MuJoCo 简化仿真的示例。

默认行为：
- 读取 `configs/urdf_mujoco_demo_config.json`
- 用你提供的 URDF 解析关节结构
- 在 MuJoCo 中生成一个简化的关节链模型
- 播放一段教学动作，并输出 CSV、曲线图、轨迹图和 GIF
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.urdf_mujoco_demo import (
    URDFKinematicChainAdapter,
    build_joint_trajectory,
    collect_urdf_replay_frames,
    load_urdf_demo_config,
    render_urdf_replay_gif,
    replay_urdf_in_viewer,
    save_urdf_replay_csv,
    save_urdf_replay_plots,
)


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="运行基于外部 URDF 的 MuJoCo 简化仿真。")
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "urdf_mujoco_demo_config.json",
        help="URDF MuJoCo 演示配置文件路径。",
    )
    parser.add_argument("--viewer", action="store_true", help="强制打开 MuJoCo viewer。")
    parser.add_argument("--no-viewer", action="store_true", help="强制关闭 MuJoCo viewer。")
    parser.add_argument("--render-offscreen", action="store_true", help="强制开启离线渲染。")
    parser.add_argument("--no-render-offscreen", action="store_true", help="强制关闭离线渲染。")
    return parser.parse_args()


def main() -> None:
    """运行 URDF 简化仿真。"""
    args = parse_args()
    config_path = args.config.resolve()
    config = load_urdf_demo_config(config_path)

    viewer_enabled = config["viewer_enabled"]
    if args.viewer:
        viewer_enabled = True
    if args.no_viewer:
        viewer_enabled = False

    offscreen_render_enabled = config["offscreen_render_enabled"]
    if args.render_offscreen:
        offscreen_render_enabled = True
    if args.no_render_offscreen:
        offscreen_render_enabled = False

    urdf_path = (config_path.parent / config["source_urdf_path"]).resolve()
    output_directory = PROJECT_ROOT / "logs" / config["output_directory_name"]

    adapter = URDFKinematicChainAdapter(source_urdf_path=urdf_path)
    build_result = adapter.build_runtime_model(
        output_directory=output_directory,
        base_height=config["base_height"],
        end_effector_offset=config["end_effector_offset"],
        default_limit=tuple(config["default_joint_limit"]),
        timestep=config["control_period_seconds"],
        dump_runtime_xml=config["dump_runtime_xml_copy"],
    )

    joint_count = len(build_result.joint_specs)
    start_joint_position = config["start_joint_position"][:joint_count]
    target_joint_position = config["target_joint_position"][:joint_count]

    joint_positions = build_joint_trajectory(
        start_joint_position=start_joint_position,
        target_joint_position=target_joint_position,
        control_period=config["control_period_seconds"],
        max_velocity=config["max_velocity"],
        max_acceleration=config["max_acceleration"],
    )
    observed_site_names = [site_name for site_name in config["observed_site_names"] if site_name in build_result.site_names]
    replay_frames = collect_urdf_replay_frames(
        model=build_result.model,
        joint_positions=joint_positions,
        control_period=config["control_period_seconds"],
        observed_site_names=observed_site_names,
    )

    joint_names = [joint_spec.name for joint_spec in build_result.joint_specs]
    output_directory.mkdir(parents=True, exist_ok=True)
    save_urdf_replay_csv(output_directory / "urdf_replay.csv", replay_frames, joint_names=joint_names)
    save_urdf_replay_plots(
        output_directory=output_directory,
        replay_frames=replay_frames,
        joint_names=joint_names,
        site_names=observed_site_names,
        title_prefix=config["plot_title_prefix"],
    )

    gif_generated = False
    if offscreen_render_enabled:
        gif_generated = render_urdf_replay_gif(
            model=build_result.model,
            replay_frames=replay_frames,
            output_gif_path=output_directory / "urdf_replay.gif",
            width=config["render_width"],
            height=config["render_height"],
            render_fps=config["render_fps"],
        )

    print("URDF MuJoCo 简化仿真完成。")
    print(f"源 URDF: {build_result.source_urdf_path}")
    print(f"关节数量: {joint_count}")
    print(f"回放步数: {len(replay_frames)}")
    print(f"CSV 输出: {output_directory / 'urdf_replay.csv'}")
    print(f"关节图输出: {output_directory / 'urdf_joint_tracking.png'}")
    print(f"site 轨迹图输出: {output_directory / 'urdf_site_trajectory.png'}")
    if offscreen_render_enabled and gif_generated:
        print(f"GIF 输出: {output_directory / 'urdf_replay.gif'}")
    elif offscreen_render_enabled:
        print("GIF 输出: 当前环境缺少 imageio/Pillow，已跳过 GIF 导出。")
    if build_result.runtime_xml_path is not None:
        print(f"运行时 MuJoCo XML: {build_result.runtime_xml_path}")

    if viewer_enabled:
        replay_urdf_in_viewer(
            model=build_result.model,
            replay_frames=replay_frames,
            playback_speed=config["playback_speed"],
            startup_pause_seconds=config.get("viewer_startup_pause_seconds", 0.8),
            keep_open_after_replay=config.get("viewer_keep_open_after_replay", True),
        )


if __name__ == "__main__":
    main()
