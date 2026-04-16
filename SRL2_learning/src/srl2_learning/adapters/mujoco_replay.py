"""SRL2 原模型 MuJoCo 回放适配层。

这个文件在整条学习链路中的位置：
- 它位于学习版最小主链路之后。
- 上游负责生成一串关节目标，这里负责把这些目标写进 MuJoCo 的 `qpos`，
  再用 viewer 或离线渲染把动作播放出来。

这个文件解决什么问题：
- 原项目已经有 MuJoCo 模型和 `qpos` 写法，但当前仓库缺失 STL 网格。
- 学习版需要在不修改 `SRL2` 的前提下，保留原 XML 结构和原 `qpos` 布局，
  做出一个能运行、能观察、能导出结果的教学回放版本。

当前边界：
- 这里不是原项目 `mujocosim.cpp` 的完整 Python 等价版。
- 第一版只复现右侧 SRAs 机械臂动作。
- 左臂和人体手臂保持中立姿态。
- 当原模型网格缺失时，会自动切换到“无网格结构回放模式”。
"""

from __future__ import annotations

import csv
import importlib.util
import json
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib.pyplot as plt
import mujoco
from matplotlib import font_manager, rcParams

from ..core.controller_pipeline import LearningRobotController, OfflinePipelineStep


def _configure_matplotlib_fonts() -> None:
    """为中文图表挑选一个本机可用字体。"""
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
    rcParams["axes.unicode_minus"] = False


def _get_child_parent_pairs(root: ET.Element) -> list[tuple[ET.Element, ET.Element]]:
    """返回 XML 树中的父子节点对。"""
    parent_child_pairs: list[tuple[ET.Element, ET.Element]] = []
    for parent in root.iter():
        for child in list(parent):
            parent_child_pairs.append((parent, child))
    return parent_child_pairs


@dataclass(slots=True)
class SRL2MujocoModelInfo:
    """原模型加载信息。"""

    source_model_path: Path
    mesh_enabled: bool
    meshless_playback_mode: bool
    missing_mesh_files: list[str]
    sanitized_xml_dump_path: Path | None
    nq: int
    njnt: int
    site_names: list[str]


@dataclass(slots=True)
class MuJoCoReplayFrame:
    """单步回放结果。"""

    step_index: int
    time_seconds: float
    right_arm_target: list[float]
    qpos: list[float]
    site_positions: dict[str, list[float]]


class SRL2MujocoModelAdapter:
    """原 `Rsras1.xml` 的运行时适配器。"""

    def __init__(self, source_model_path: str | Path) -> None:
        self.source_model_path = Path(source_model_path)

    def _read_xml_root(self) -> ET.Element:
        """读取原始 XML 并解析为元素树。"""
        return ET.fromstring(self.source_model_path.read_text(encoding="utf-8", errors="ignore"))

    def _detect_missing_mesh_files(self, root: ET.Element) -> list[str]:
        """检查原模型引用的 mesh 是否在当前环境中可用。"""
        compiler = root.find("compiler")
        meshdir = compiler.get("meshdir") if compiler is not None else None
        mesh_elements = root.findall("./asset/mesh")

        missing_files: list[str] = []
        for mesh_element in mesh_elements:
            mesh_file = mesh_element.get("file")
            if not mesh_file:
                continue

            candidate_paths: list[Path] = []
            if meshdir:
                candidate_paths.append(Path(meshdir) / mesh_file)
            candidate_paths.append(self.source_model_path.parent / mesh_file)

            if not any(candidate_path.exists() for candidate_path in candidate_paths):
                missing_files.append(mesh_file)

        return missing_files

    def _build_meshless_xml(self, root: ET.Element) -> str:
        """删除 mesh 和相关 contact，得到可运行的无网格 XML。"""
        compiler = root.find("compiler")
        if compiler is not None and "meshdir" in compiler.attrib:
            compiler.attrib.pop("meshdir")

        asset = root.find("asset")
        if asset is not None:
            for mesh_element in list(asset.findall("mesh")):
                asset.remove(mesh_element)

        for parent, child in _get_child_parent_pairs(root):
            if child.tag == "geom" and "mesh" in child.attrib:
                parent.remove(child)

        contact = root.find("contact")
        if contact is not None:
            root.remove(contact)

        return ET.tostring(root, encoding="unicode")

    def load_model(
        self,
        debug_output_directory: Path | None = None,
        dump_sanitized_xml_copy: bool = False,
    ) -> tuple[mujoco.MjModel, SRL2MujocoModelInfo]:
        """加载 MuJoCo 模型，并在必要时切到无网格结构回放模式。"""
        root = self._read_xml_root()
        missing_mesh_files = self._detect_missing_mesh_files(root)

        sanitized_xml_dump_path: Path | None = None
        if not missing_mesh_files:
            model = mujoco.MjModel.from_xml_path(str(self.source_model_path))
            mesh_enabled = True
            meshless_playback_mode = False
        else:
            sanitized_xml = self._build_meshless_xml(root)
            if dump_sanitized_xml_copy and debug_output_directory is not None:
                debug_output_directory.mkdir(parents=True, exist_ok=True)
                sanitized_xml_dump_path = debug_output_directory / "Rsras1_meshless_runtime.xml"
                sanitized_xml_dump_path.write_text(sanitized_xml, encoding="utf-8")
            model = mujoco.MjModel.from_xml_string(sanitized_xml)
            mesh_enabled = False
            meshless_playback_mode = True

        site_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, site_index) or f"site_{site_index}"
            for site_index in range(model.nsite)
        ]

        return model, SRL2MujocoModelInfo(
            source_model_path=self.source_model_path,
            mesh_enabled=mesh_enabled,
            meshless_playback_mode=meshless_playback_mode,
            missing_mesh_files=missing_mesh_files,
            sanitized_xml_dump_path=sanitized_xml_dump_path,
            nq=model.nq,
            njnt=model.njnt,
            site_names=site_names,
        )


class SRL2QposWriter:
    """按原 `main.cpp` 布局写入 `qpos` 的适配器。"""

    def __init__(self) -> None:
        self.expected_qpos_length = 29

    def build_qpos(
        self,
        right_arm_joint_positions: Iterable[float],
        left_arm_joint_positions: Iterable[float] | None = None,
    ) -> list[float]:
        """根据右臂关节目标构造一帧完整 `qpos`。"""
        right_arm = [float(value) for value in right_arm_joint_positions]
        if len(right_arm) != 6:
            raise ValueError("第一版 MuJoCo 回放要求右臂输入恰好为 6 维关节角。")

        left_arm = [0.0] * 6 if left_arm_joint_positions is None else [float(value) for value in left_arm_joint_positions]
        if len(left_arm) != 6:
            raise ValueError("左臂输入必须为 6 维，或直接留空使用中立姿态。")

        qpos = [0.0] * self.expected_qpos_length
        qpos[0:3] = [0.0, 0.0, 0.0]
        qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        qpos[7:13] = right_arm
        qpos[13:19] = left_arm
        qpos[19:23] = [1.0, 0.0, 0.0, 0.0]
        qpos[23] = 0.0
        qpos[24:28] = [1.0, 0.0, 0.0, 0.0]
        qpos[28] = 0.0
        return qpos

    def apply_frame(
        self,
        data: mujoco.MjData,
        right_arm_joint_positions: Iterable[float],
        left_arm_joint_positions: Iterable[float] | None = None,
    ) -> list[float]:
        """把一帧 `qpos` 直接写入 MuJoCo 数据对象。"""
        qpos = self.build_qpos(
            right_arm_joint_positions=right_arm_joint_positions,
            left_arm_joint_positions=left_arm_joint_positions,
        )
        data.qpos[:] = qpos
        return qpos


def build_minimal_right_arm_motion(
    config: dict[str, Any],
    config_directory: Path,
) -> list[OfflinePipelineStep]:
    """复用学习版现有最小主链路，生成右臂动作序列。"""
    controller = LearningRobotController(
        control_period=config["control_period_seconds"],
        joint_dof=config["joint_dof"],
        command_dof=config["command_dof"],
    )

    reference_model_path = (config_directory / config["reference_model_path"]).resolve()
    controller.load_reference_model(
        model_path=str(reference_model_path),
        end_link_name=config["end_link_name"],
        tool_offset=config["tool_offset"],
    )

    return controller.run_joint_space_motion(
        start_joint_command=config["start_joint_command"],
        target_joint_command=config["target_joint_command"],
        max_velocity=config["max_velocity"],
        max_acceleration=config["max_acceleration"],
    )


def collect_replay_frames(
    model: mujoco.MjModel,
    pipeline_steps: list[OfflinePipelineStep],
    qpos_writer: SRL2QposWriter,
    control_period: float,
    observed_site_names: list[str],
) -> list[MuJoCoReplayFrame]:
    """把学习版动作序列变成 MuJoCo 回放帧。"""
    data = mujoco.MjData(model)
    site_ids = {
        site_name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        for site_name in observed_site_names
    }

    replay_frames: list[MuJoCoReplayFrame] = []
    for step in pipeline_steps:
        right_arm_target = step.desired_joint_position[:6]
        qpos = qpos_writer.apply_frame(data, right_arm_joint_positions=right_arm_target)
        mujoco.mj_forward(model, data)

        site_positions: dict[str, list[float]] = {}
        for site_name, site_id in site_ids.items():
            if site_id >= 0:
                site_positions[site_name] = [float(value) for value in data.site_xpos[site_id]]

        replay_frames.append(
            MuJoCoReplayFrame(
                step_index=step.step_index,
                time_seconds=step.step_index * control_period,
                right_arm_target=right_arm_target.copy(),
                qpos=[float(value) for value in qpos],
                site_positions=site_positions,
            )
        )

    return replay_frames


def save_replay_csv(output_csv_path: Path, replay_frames: list[MuJoCoReplayFrame]) -> None:
    """保存 MuJoCo 回放的时序 CSV。"""
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)
    headers = ["step_index", "time_seconds"]
    headers.extend(f"right_arm_target_{joint_index + 1}" for joint_index in range(6))
    headers.extend(f"qpos_{qpos_index}" for qpos_index in range(29))

    site_names = sorted({site_name for frame in replay_frames for site_name in frame.site_positions})
    for site_name in site_names:
        headers.extend([f"{site_name}_x", f"{site_name}_y", f"{site_name}_z"])

    with output_csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(headers)
        for frame in replay_frames:
            row: list[float | int | str] = [frame.step_index, frame.time_seconds]
            row.extend(frame.right_arm_target)
            row.extend(frame.qpos)
            for site_name in site_names:
                if site_name in frame.site_positions:
                    row.extend(frame.site_positions[site_name])
                else:
                    row.extend(["", "", ""])
            writer.writerow(row)


def _create_joint_plot(
    output_plot_path: Path,
    replay_frames: list[MuJoCoReplayFrame],
    title_prefix: str,
) -> None:
    """绘制右臂六关节目标角与 MuJoCo qpos 对照图。"""
    _configure_matplotlib_fonts()
    output_plot_path.parent.mkdir(parents=True, exist_ok=True)

    times = [frame.time_seconds for frame in replay_frames]
    figure, axes = plt.subplots(6, 1, figsize=(12, 15), sharex=True)
    for joint_index, axis in enumerate(axes):
        target_values = [frame.right_arm_target[joint_index] for frame in replay_frames]
        qpos_values = [frame.qpos[7 + joint_index] for frame in replay_frames]
        axis.plot(times, target_values, label="主链路目标角", linewidth=2.0)
        axis.plot(times, qpos_values, label="MuJoCo qpos", linestyle="--", linewidth=1.8)
        axis.set_ylabel(f"关节{joint_index + 1}\n角度(rad)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")

    axes[-1].set_xlabel("时间 (s)")
    figure.suptitle(f"{title_prefix} - 右臂关节目标与 qpos 对照", fontsize=14)
    figure.tight_layout()
    figure.savefig(output_plot_path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def _create_site_trajectory_plot(
    output_plot_path: Path,
    replay_frames: list[MuJoCoReplayFrame],
    title_prefix: str,
    site_names: list[str],
) -> None:
    """绘制关键 site 的 3D 轨迹图。"""
    _configure_matplotlib_fonts()
    output_plot_path.parent.mkdir(parents=True, exist_ok=True)

    available_site_names = [site_name for site_name in site_names if any(site_name in frame.site_positions for frame in replay_frames)]
    if not available_site_names:
        return

    figure = plt.figure(figsize=(7 * len(available_site_names), 6))
    for subplot_index, site_name in enumerate(available_site_names, start=1):
        axis = figure.add_subplot(1, len(available_site_names), subplot_index, projection="3d")
        positions = [frame.site_positions[site_name] for frame in replay_frames if site_name in frame.site_positions]
        xs = [position[0] for position in positions]
        ys = [position[1] for position in positions]
        zs = [position[2] for position in positions]
        axis.plot(xs, ys, zs, linewidth=2.0, label=site_name)
        axis.scatter(xs[0], ys[0], zs[0], color="green", label="起点")
        axis.scatter(xs[-1], ys[-1], zs[-1], color="red", label="终点")
        axis.set_title(site_name)
        axis.set_xlabel("x")
        axis.set_ylabel("y")
        axis.set_zlabel("z")
        axis.legend(loc="best")

    figure.suptitle(f"{title_prefix} - 关键 site 轨迹", fontsize=14)
    figure.tight_layout()
    figure.savefig(output_plot_path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def save_replay_plots(
    output_directory: Path,
    replay_frames: list[MuJoCoReplayFrame],
    title_prefix: str,
    site_names: list[str],
) -> None:
    """保存 MuJoCo 回放相关图表。"""
    _create_joint_plot(
        output_plot_path=output_directory / "right_arm_joint_tracking.png",
        replay_frames=replay_frames,
        title_prefix=title_prefix,
    )
    _create_site_trajectory_plot(
        output_plot_path=output_directory / "site_trajectory.png",
        replay_frames=replay_frames,
        title_prefix=title_prefix,
        site_names=site_names,
    )


def render_offscreen_gif(
    model: mujoco.MjModel,
    replay_frames: list[MuJoCoReplayFrame],
    output_gif_path: Path,
    width: int,
    height: int,
    render_fps: int,
) -> bool:
    """离线渲染 GIF。

    返回值：
    - `True`：GIF 成功生成。
    - `False`：当前环境缺失 GIF 编码依赖，已跳过 GIF 导出。
    """
    output_gif_path.parent.mkdir(parents=True, exist_ok=True)
    data = mujoco.MjData(model)
    framebuffer_width = int(model.vis.global_.offwidth)
    framebuffer_height = int(model.vis.global_.offheight)
    render_width = min(int(width), framebuffer_width)
    render_height = min(int(height), framebuffer_height)
    renderer = mujoco.Renderer(model, width=render_width, height=render_height)

    if len(replay_frames) >= 2:
        simulated_frequency = 1.0 / max(replay_frames[1].time_seconds - replay_frames[0].time_seconds, 1e-6)
    else:
        simulated_frequency = float(render_fps)
    frame_stride = max(1, int(round(simulated_frequency / max(render_fps, 1))))

    rendered_images = []
    for frame in replay_frames[::frame_stride]:
        data.qpos[:] = frame.qpos
        mujoco.mj_forward(model, data)
        renderer.update_scene(data)
        rendered_images.append(renderer.render())

    renderer.close()
    imageio_available = importlib.util.find_spec("imageio") is not None
    pillow_available = importlib.util.find_spec("PIL") is not None

    if imageio_available:
        import imageio.v2 as imageio

        imageio.mimsave(output_gif_path, rendered_images, fps=render_fps)
        return True

    if pillow_available:
        from PIL import Image

        pil_images = [Image.fromarray(image) for image in rendered_images]
        pil_images[0].save(
            output_gif_path,
            save_all=True,
            append_images=pil_images[1:],
            duration=max(1, int(1000 / max(render_fps, 1))),
            loop=0,
        )
        return True

    return False


def replay_in_viewer(
    model: mujoco.MjModel,
    replay_frames: list[MuJoCoReplayFrame],
    playback_speed: float,
    startup_pause_seconds: float = 0.8,
    keep_open_after_replay: bool = True,
) -> None:
    """用 MuJoCo viewer 实时播放一遍动作。

    设计目标：
    - viewer 至少要有足够时间真正弹出来
    - 动作播完后默认停在最后一帧，而不是立刻退出
    """
    import mujoco.viewer as viewer

    data = mujoco.MjData(model)
    with viewer.launch_passive(model, data) as viewer_handle:
        viewer_handle.sync()
        if startup_pause_seconds > 0:
            time.sleep(startup_pause_seconds)
        for frame in replay_frames:
            if not viewer_handle.is_running():
                return
            data.qpos[:] = frame.qpos
            mujoco.mj_forward(model, data)
            viewer_handle.sync()
            if playback_speed > 0:
                if len(replay_frames) >= 2:
                    step_interval = replay_frames[1].time_seconds - replay_frames[0].time_seconds
                else:
                    step_interval = 0.03
                time.sleep(step_interval / playback_speed)

        if keep_open_after_replay:
            while viewer_handle.is_running():
                viewer_handle.sync()
                time.sleep(0.03)


def load_mujoco_demo_config(config_path: Path) -> dict[str, Any]:
    """读取 MuJoCo 演示配置。"""
    return json.loads(config_path.read_text(encoding="utf-8"))
