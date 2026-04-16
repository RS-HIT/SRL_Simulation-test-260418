"""SRL2 原模型 MuJoCo 回放示例。

默认行为：
- 读取 `configs/mujoco_demo_config.json`
- 用学习版最小主链路生成右臂动作
- 按原 `main.cpp` 的 `qpos` 布局写入 MuJoCo
- 输出 CSV、关节图、site 轨迹图和 GIF

可选行为：
- 通过 `--viewer` 打开实时 MuJoCo viewer 回放
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.mujoco_replay import (
    SRL2MujocoModelAdapter,
    SRL2QposWriter,
    build_minimal_right_arm_motion,
    collect_replay_frames,
    load_mujoco_demo_config,
    render_offscreen_gif,
    replay_in_viewer,
    save_replay_csv,
    save_replay_plots,
)


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="运行 SRL2 原模型 MuJoCo 教学回放。")
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "mujoco_demo_config.json",
        help="MuJoCo 演示配置文件路径。",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="强制打开 MuJoCo viewer 实时回放。",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="强制关闭 MuJoCo viewer。",
    )
    parser.add_argument(
        "--render-offscreen",
        action="store_true",
        help="强制开启离线渲染。",
    )
    parser.add_argument(
        "--no-render-offscreen",
        action="store_true",
        help="强制关闭离线渲染。",
    )
    return parser.parse_args()


def main() -> None:
    """运行 MuJoCo 回放示例。"""
    args = parse_args()
    config_path = args.config.resolve()
    config = load_mujoco_demo_config(config_path)
    if config["arm_side"] != "right":
        raise NotImplementedError("第一版 MuJoCo 回放当前只支持右侧 SRAs 机械臂。")

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

    source_model_path = (config_path.parent / config["source_model_path"]).resolve()
    output_directory = PROJECT_ROOT / "logs" / config["output_directory_name"]

    adapter = SRL2MujocoModelAdapter(source_model_path=source_model_path)
    model, model_info = adapter.load_model(
        debug_output_directory=output_directory,
        dump_sanitized_xml_copy=config["dump_sanitized_xml_copy"],
    )

    pipeline_steps = build_minimal_right_arm_motion(
        config=config,
        config_directory=config_path.parent,
    )
    qpos_writer = SRL2QposWriter()
    replay_frames = collect_replay_frames(
        model=model,
        pipeline_steps=pipeline_steps,
        qpos_writer=qpos_writer,
        control_period=config["control_period_seconds"],
        observed_site_names=config["observed_site_names"],
    )

    output_directory.mkdir(parents=True, exist_ok=True)
    save_replay_csv(output_directory / "mujoco_replay.csv", replay_frames)
    save_replay_plots(
        output_directory=output_directory,
        replay_frames=replay_frames,
        title_prefix=config["plot_title_prefix"],
        site_names=config["observed_site_names"],
    )

    gif_generated = False
    if offscreen_render_enabled:
        gif_generated = render_offscreen_gif(
            model=model,
            replay_frames=replay_frames,
            output_gif_path=output_directory / "mujoco_replay.gif",
            width=config["render_width"],
            height=config["render_height"],
            render_fps=config["render_fps"],
        )

    print("MuJoCo 原模型回放完成。")
    print(f"源模型: {model_info.source_model_path}")
    print(f"是否保留 mesh: {model_info.mesh_enabled}")
    print(f"是否无网格结构回放模式: {model_info.meshless_playback_mode}")
    if model_info.missing_mesh_files:
        print(f"缺失 mesh 文件数量: {len(model_info.missing_mesh_files)}")
    print(f"qpos 长度: {model_info.nq}")
    print(f"关节数量: {model_info.njnt}")
    print(f"回放步数: {len(replay_frames)}")
    print(f"CSV 输出: {output_directory / 'mujoco_replay.csv'}")
    print(f"关节图输出: {output_directory / 'right_arm_joint_tracking.png'}")
    print(f"site 轨迹图输出: {output_directory / 'site_trajectory.png'}")
    if offscreen_render_enabled and gif_generated:
        print(f"GIF 输出: {output_directory / 'mujoco_replay.gif'}")
    elif offscreen_render_enabled:
        print("GIF 输出: 当前环境缺少 imageio/Pillow，已跳过 GIF 导出，但 CSV 和图表已正常生成。")
    if model_info.sanitized_xml_dump_path is not None:
        print(f"运行时清洗 XML: {model_info.sanitized_xml_dump_path}")

    if viewer_enabled:
        replay_in_viewer(
            model=model,
            replay_frames=replay_frames,
            playback_speed=config["playback_speed"],
            startup_pause_seconds=config.get("viewer_startup_pause_seconds", 0.8),
            keep_open_after_replay=config.get("viewer_keep_open_after_replay", True),
        )


if __name__ == "__main__":
    main()
