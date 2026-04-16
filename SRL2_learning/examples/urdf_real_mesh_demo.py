"""使用真实 STL 网格运行 URDF MuJoCo 仿真。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.adapters.urdf_mujoco_demo import (
    build_joint_trajectory,
    collect_urdf_replay_frames,
    render_urdf_replay_gif,
    replay_urdf_in_viewer,
    save_urdf_replay_csv,
    save_urdf_replay_plots,
)
from srl2_learning.adapters.urdf_real_mesh_mujoco import (
    URDFRealMeshPreparationAdapter,
    load_real_mesh_demo_config,
    load_real_mesh_mujoco_model,
)


def parse_args() -> argparse.Namespace:
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(description="运行基于真实 STL 网格的 URDF MuJoCo 仿真。")
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_ROOT / "configs" / "urdf_real_mesh_demo_config.json",
        help="真实网格仿真配置文件路径。",
    )
    parser.add_argument("--prepare-only", action="store_true", help="只做网格预处理，不启动仿真。")
    parser.add_argument("--force-rebuild", action="store_true", help="强制重新减面并重建运行时 URDF。")
    parser.add_argument("--viewer", action="store_true", help="强制打开 MuJoCo viewer。")
    parser.add_argument("--no-viewer", action="store_true", help="强制关闭 MuJoCo viewer。")
    parser.add_argument("--render-offscreen", action="store_true", help="强制开启离线渲染。")
    parser.add_argument("--no-render-offscreen", action="store_true", help="强制关闭离线渲染。")
    return parser.parse_args()


def main() -> None:
    """运行真实网格 MuJoCo 仿真。"""
    args = parse_args()
    config_path = args.config.resolve()
    config = load_real_mesh_demo_config(config_path)

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

    source_urdf_path = (config_path.parent / config["source_urdf_path"]).resolve()
    processed_model_root = (PROJECT_ROOT / config["processed_model_root"]).resolve()
    output_directory = PROJECT_ROOT / "logs" / config["output_directory_name"]

    adapter = URDFRealMeshPreparationAdapter(
        source_urdf_path=source_urdf_path,
        processed_model_root=processed_model_root,
    )
    preparation_result = adapter.prepare_assets(
        mesh_face_limits=config["mesh_face_limits"],
        base_position=config["base_position"],
        base_euler=config["base_euler"],
        ground_height=config["ground_height"],
        add_ground_plane=config.get("add_ground_plane", False),
        default_joint_limit=tuple(config["default_joint_limit"]),
        force_rebuild=args.force_rebuild,
    )

    print("真实网格预处理完成。")
    print(f"源 URDF: {preparation_result.source_urdf_path}")
    print(f"运行时 URDF: {preparation_result.runtime_urdf_path}")
    print(f"运行时 MJCF: {preparation_result.runtime_mjcf_path}")
    for mesh_info in preparation_result.mesh_infos:
        print(
            f"mesh {mesh_info.mesh_name}: "
            f"{mesh_info.source_face_count} -> {mesh_info.output_face_count} faces"
        )

    if args.prepare_only:
        return

    model = load_real_mesh_mujoco_model(preparation_result.runtime_mjcf_path)

    joint_count = model.nq
    start_joint_position = config["start_joint_position"][:joint_count]
    target_joint_position = config["target_joint_position"][:joint_count]
    joint_positions = build_joint_trajectory(
        start_joint_position=start_joint_position,
        target_joint_position=target_joint_position,
        control_period=config["control_period_seconds"],
        max_velocity=config["max_velocity"],
        max_acceleration=config["max_acceleration"],
    )

    replay_frames = collect_urdf_replay_frames(
        model=model,
        joint_positions=joint_positions,
        control_period=config["control_period_seconds"],
        observed_site_names=config["observed_site_names"],
    )

    output_directory.mkdir(parents=True, exist_ok=True)
    joint_names = [f"joint_{index + 1}" for index in range(joint_count)]
    save_urdf_replay_csv(output_directory / "urdf_real_mesh_replay.csv", replay_frames, joint_names=joint_names)
    save_urdf_replay_plots(
        output_directory=output_directory,
        replay_frames=replay_frames,
        joint_names=joint_names,
        site_names=config["observed_site_names"],
        title_prefix=config["plot_title_prefix"],
    )

    gif_generated = False
    if offscreen_render_enabled:
        gif_generated = render_urdf_replay_gif(
            model=model,
            replay_frames=replay_frames,
            output_gif_path=output_directory / "urdf_real_mesh_replay.gif",
            width=config["render_width"],
            height=config["render_height"],
            render_fps=config["render_fps"],
            camera_lookat=config.get("viewer_camera_lookat"),
            camera_distance=config.get("viewer_camera_distance"),
            camera_azimuth=config.get("viewer_camera_azimuth"),
            camera_elevation=config.get("viewer_camera_elevation"),
        )

    print("真实网格 MuJoCo 仿真完成。")
    print(f"CSV 输出: {output_directory / 'urdf_real_mesh_replay.csv'}")
    print(f"关节图输出: {output_directory / 'urdf_joint_tracking.png'}")
    print(f"site 轨迹图输出: {output_directory / 'urdf_site_trajectory.png'}")
    if offscreen_render_enabled and gif_generated:
        print(f"GIF 输出: {output_directory / 'urdf_real_mesh_replay.gif'}")
    elif offscreen_render_enabled:
        print("GIF 输出: 当前环境缺少 imageio/Pillow，已跳过 GIF 导出。")

    if viewer_enabled:
        replay_urdf_in_viewer(
            model=model,
            replay_frames=replay_frames,
            playback_speed=config["playback_speed"],
            startup_pause_seconds=config.get("viewer_startup_pause_seconds", 0.8),
            keep_open_after_replay=config.get("viewer_keep_open_after_replay", True),
            camera_lookat=config.get("viewer_camera_lookat"),
            camera_distance=config.get("viewer_camera_distance"),
            camera_azimuth=config.get("viewer_camera_azimuth"),
            camera_elevation=config.get("viewer_camera_elevation"),
        )


if __name__ == "__main__":
    main()
