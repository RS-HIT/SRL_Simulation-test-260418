"""基于外部 URDF 的 MuJoCo 简化仿真适配层。

这个文件在整条学习链路里的位置：
- 它不替代原始 `SRL2/model/Rsras1.xml` 回放链路。
- 它是单独新增的一条“用外部 URDF 做简化 MuJoCo 仿真”的教学链路。

它解决什么问题：
- 你提供的 URDF 带有机械臂结构信息，但原始 `package://` 网格路径和高面数 STL
  并不适合直接让 MuJoCo 在当前环境里稳定导入。
- 这里改为“读取 URDF 的关节结构”，然后在 MuJoCo 里生成一个简化的几何模型，
  先把动作、层级关系、关节姿态和可视化过程跑通。

当前边界：
- 这不是对原 SolidWorks 网格的完整还原。
- 这不是原真机控制程序的等价实现。
- 这里更像“拿 URDF 做骨架参考，再生成一个稳定可演示的 MuJoCo 教学模型”。
"""

from __future__ import annotations

import csv
import json
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib.pyplot as plt
import mujoco
from matplotlib import font_manager, rcParams

from ..core.trajectory_planner import LinearInterpolationTrajectoryPlanner


def _configure_matplotlib_fonts() -> None:
    """为中文图表选择一个本机可用字体。"""
    available_font_names = {font.name for font in font_manager.fontManager.ttflist}
    preferred_fonts = ["Microsoft YaHei", "SimHei", "SimSun", "Microsoft JhengHei"]
    for font_name in preferred_fonts:
        if font_name in available_font_names:
            rcParams["font.sans-serif"] = [font_name]
            rcParams["axes.unicode_minus"] = False
            return
    rcParams["axes.unicode_minus"] = False


def _vector_text(values: Iterable[float]) -> str:
    """把数字列表转成 MuJoCo XML 需要的空格分隔文本。"""
    return " ".join(f"{float(value):.6f}" for value in values)


def _vector_norm(values: Iterable[float]) -> float:
    """计算一个向量的长度。"""
    return math.sqrt(sum(float(value) * float(value) for value in values))


def apply_default_like_visual_preset(model: mujoco.MjModel) -> None:
    """把 URDF 导入后的场景调成更接近 MuJoCo 默认 viewer 的观感。

    为什么需要它：
    - 直接由 URDF 导入的模型通常只有几何体，没有 MJCF 里常见的完整场景配置。
    - 结果就是 viewer 虽然能显示模型，但视觉上容易偏暗，背景也比较“空”。
    - 这里不伪造复杂美术场景，只把头灯和远景颜色调到更容易观察机械臂的程度。
    """
    model.vis.headlight.ambient[:] = [0.35, 0.35, 0.35]
    model.vis.headlight.diffuse[:] = [0.85, 0.85, 0.85]
    model.vis.headlight.specular[:] = [0.25, 0.25, 0.25]
    model.vis.rgba.haze[:] = [0.92, 0.95, 1.00, 1.00]


def configure_free_camera(
    camera: mujoco.MjvCamera,
    lookat: list[float] | None = None,
    distance: float | None = None,
    azimuth: float | None = None,
    elevation: float | None = None,
) -> None:
    """给 free camera 一个稳定、易观察的初始视角。"""
    if lookat is not None:
        camera.lookat[:] = [float(value) for value in lookat]
    if distance is not None:
        camera.distance = float(distance)
    if azimuth is not None:
        camera.azimuth = float(azimuth)
    if elevation is not None:
        camera.elevation = float(elevation)


@dataclass(slots=True)
class URDFJointSpec:
    """URDF 中单个关节的关键信息。"""

    name: str
    parent_link: str
    child_link: str
    origin_xyz: list[float]
    axis_xyz: list[float]
    lower_limit: float
    upper_limit: float


@dataclass(slots=True)
class URDFMuJoCoFrame:
    """URDF 仿真中的单步结果。"""

    step_index: int
    time_seconds: float
    joint_target: list[float]
    qpos: list[float]
    site_positions: dict[str, list[float]]


@dataclass(slots=True)
class URDFModelBuildResult:
    """URDF -> MuJoCo 运行时构建结果。"""

    source_urdf_path: Path
    runtime_xml_path: Path | None
    joint_specs: list[URDFJointSpec]
    model: mujoco.MjModel
    site_names: list[str]


class URDFKinematicChainAdapter:
    """把 URDF 关节链转换成 MuJoCo 简化模型。

    这个类为什么存在：
    - 你给的 URDF 里确实有机械臂层级和关节定义，这是最有价值的部分。
    - 但原 STL 网格太重、路径也不适配当前环境。
    - 所以这个类负责“保留结构，简化外观”。
    """

    def __init__(self, source_urdf_path: str | Path) -> None:
        self.source_urdf_path = Path(source_urdf_path)

    def parse_joint_specs(
        self,
        default_limit: tuple[float, float],
    ) -> list[URDFJointSpec]:
        """读取 URDF 中的关节链。

        什么时候调用它：
        - 在生成 MuJoCo 模型之前调用。
        - 它先把 URDF 里的 joint/origin/axis/limit 解析出来，
          后面的 XML 生成和动作回放都依赖这个结果。
        """
        root = ET.parse(self.source_urdf_path).getroot()
        joint_specs: list[URDFJointSpec] = []

        for joint_element in root.findall("joint"):
            if joint_element.get("type") != "revolute":
                continue

            origin_element = joint_element.find("origin")
            axis_element = joint_element.find("axis")
            limit_element = joint_element.find("limit")
            parent_element = joint_element.find("parent")
            child_element = joint_element.find("child")

            if (
                origin_element is None
                or axis_element is None
                or limit_element is None
                or parent_element is None
                or child_element is None
            ):
                continue

            origin_xyz = [float(value) for value in origin_element.get("xyz", "0 0 0").split()]
            axis_xyz = [float(value) for value in axis_element.get("xyz", "0 0 1").split()]
            lower_limit = float(limit_element.get("lower", default_limit[0]))
            upper_limit = float(limit_element.get("upper", default_limit[1]))

            # 这个 SolidWorks 导出的 URDF 把 3 个关节的上下限都写成了 0。
            # 对 MuJoCo 来说，这相当于“你定义了关节，但又把它锁死了”。
            # 教学版这里把这种明显无效的零区间，替换成一个保守的默认范围。
            if abs(lower_limit - upper_limit) < 1e-9:
                lower_limit, upper_limit = default_limit

            joint_specs.append(
                URDFJointSpec(
                    name=joint_element.get("name", "unnamed_joint"),
                    parent_link=parent_element.get("link", "unknown_parent"),
                    child_link=child_element.get("link", "unknown_child"),
                    origin_xyz=origin_xyz,
                    axis_xyz=axis_xyz,
                    lower_limit=lower_limit,
                    upper_limit=upper_limit,
                )
            )

        if not joint_specs:
            raise ValueError("给定 URDF 中没有解析出可用的 revolute 关节。")
        return joint_specs

    def build_runtime_model(
        self,
        output_directory: Path,
        base_height: float,
        end_effector_offset: list[float],
        default_limit: tuple[float, float],
        timestep: float,
        dump_runtime_xml: bool = True,
    ) -> URDFModelBuildResult:
        """把 URDF 关节结构转换成 MuJoCo 简化模型。"""
        joint_specs = self.parse_joint_specs(default_limit=default_limit)

        mujoco_root = ET.Element("mujoco", model=f"{self.source_urdf_path.stem}_teaching_demo")
        ET.SubElement(mujoco_root, "compiler", angle="radian", autolimits="true")
        ET.SubElement(mujoco_root, "option", timestep=f"{float(timestep):.6f}", gravity="0 0 -9.81")
        ET.SubElement(
            mujoco_root,
            "visual",
        )
        asset = ET.SubElement(mujoco_root, "asset")
        ET.SubElement(asset, "texture", name="grid", type="2d", builtin="checker", rgb1="0.85 0.85 0.85", rgb2="0.75 0.75 0.75", width="256", height="256")
        ET.SubElement(asset, "material", name="floor", texture="grid", texrepeat="4 4", reflectance="0.1")

        worldbody = ET.SubElement(mujoco_root, "worldbody")
        ET.SubElement(worldbody, "geom", name="floor", type="plane", size="3 3 0.1", pos="0 0 0", material="floor")
        ET.SubElement(worldbody, "light", name="main_light", pos="0 0 3", dir="0 0 -1", diffuse="1 1 1")
        ET.SubElement(worldbody, "camera", name="overview", pos="2.2 -2.2 1.5", xyaxes="0.707 0.707 0 -0.25 0.25 0.935")

        base_body = ET.SubElement(worldbody, "body", name="base", pos=f"0 0 {float(base_height):.6f}")
        ET.SubElement(base_body, "geom", name="base_geom", type="box", size="0.06 0.06 0.04", rgba="0.2 0.2 0.25 1")
        ET.SubElement(base_body, "site", name="base_site", pos="0 0 0", size="0.01", rgba="1 0.8 0 1")

        current_body = base_body
        for index, joint_spec in enumerate(joint_specs):
            child_body = ET.SubElement(
                current_body,
                "body",
                name=joint_spec.child_link,
                pos=_vector_text(joint_spec.origin_xyz),
            )
            ET.SubElement(
                child_body,
                "joint",
                name=joint_spec.name,
                type="hinge",
                axis=_vector_text(joint_spec.axis_xyz),
                range=_vector_text([joint_spec.lower_limit, joint_spec.upper_limit]),
                damping="0.3",
            )
            ET.SubElement(
                child_body,
                "geom",
                name=f"{joint_spec.child_link}_joint_marker",
                type="sphere",
                size="0.025",
                rgba="0.1 0.5 0.9 1",
            )

            if index + 1 < len(joint_specs):
                next_offset = joint_specs[index + 1].origin_xyz
            else:
                next_offset = end_effector_offset

            # 这一步为什么要这样做：
            # - URDF 里提供了“下一个关节相对当前连杆原点的位置”。
            # - 教学版用这个偏移量画一根 capsule，当作简化连杆。
            # - 这样虽然不是原始 STL 外观，但连杆长度和层级关系能看清。
            if _vector_norm(next_offset) > 1e-6:
                ET.SubElement(
                    child_body,
                    "geom",
                    name=f"{joint_spec.child_link}_link_geom",
                    type="capsule",
                    fromto=f"0 0 0 {_vector_text(next_offset)}",
                    size="0.02",
                    rgba="0.75 0.8 0.88 1",
                )

            ET.SubElement(child_body, "site", name=f"{joint_spec.child_link}_site", pos="0 0 0", size="0.012", rgba="0.9 0.3 0.1 1")
            current_body = child_body

        ET.SubElement(
            current_body,
            "site",
            name="tool_site",
            pos=_vector_text(end_effector_offset),
            size="0.018",
            rgba="0 1 0 1",
        )
        ET.SubElement(
            current_body,
            "geom",
            name="tool_tip_geom",
            type="sphere",
            pos=_vector_text(end_effector_offset),
            size="0.028",
            rgba="0 0.8 0.2 1",
        )

        actuator = ET.SubElement(mujoco_root, "actuator")
        for joint_spec in joint_specs:
            ET.SubElement(actuator, "position", name=f"{joint_spec.name}_position", joint=joint_spec.name, kp="25")

        runtime_xml = ET.tostring(mujoco_root, encoding="unicode")
        runtime_xml_path = None
        if dump_runtime_xml:
            output_directory.mkdir(parents=True, exist_ok=True)
            runtime_xml_path = output_directory / f"{self.source_urdf_path.stem}_teaching_runtime.xml"
            runtime_xml_path.write_text(runtime_xml, encoding="utf-8")

        model = mujoco.MjModel.from_xml_string(runtime_xml)
        site_names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, site_index) or f"site_{site_index}"
            for site_index in range(model.nsite)
        ]

        return URDFModelBuildResult(
            source_urdf_path=self.source_urdf_path,
            runtime_xml_path=runtime_xml_path,
            joint_specs=joint_specs,
            model=model,
            site_names=site_names,
        )


def build_joint_trajectory(
    start_joint_position: list[float],
    target_joint_position: list[float],
    control_period: float,
    max_velocity: float,
    max_acceleration: float,
) -> list[list[float]]:
    """用学习版线性规划器生成一段关节轨迹。"""
    planner = LinearInterpolationTrajectoryPlanner()
    planner.initialize(dof_count=len(target_joint_position), control_period=control_period)
    planner.plan_joint_position(
        current_position=start_joint_position,
        target_position=target_joint_position,
        max_velocity=max_velocity,
        max_acceleration=max_acceleration,
    )

    positions: list[list[float]] = []
    while True:
        result = planner.step()
        positions.append(result.position)
        if result.is_finished:
            break
    return positions


def collect_urdf_replay_frames(
    model: mujoco.MjModel,
    joint_positions: list[list[float]],
    control_period: float,
    observed_site_names: list[str],
) -> list[URDFMuJoCoFrame]:
    """把关节轨迹变成 MuJoCo 回放帧。"""
    data = mujoco.MjData(model)
    site_ids = {
        site_name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        for site_name in observed_site_names
    }

    replay_frames: list[URDFMuJoCoFrame] = []
    for step_index, joint_target in enumerate(joint_positions):
        data.qpos[:] = joint_target
        mujoco.mj_forward(model, data)

        site_positions: dict[str, list[float]] = {}
        for site_name, site_id in site_ids.items():
            if site_id >= 0:
                site_positions[site_name] = [float(value) for value in data.site_xpos[site_id]]

        replay_frames.append(
            URDFMuJoCoFrame(
                step_index=step_index,
                time_seconds=step_index * control_period,
                joint_target=[float(value) for value in joint_target],
                qpos=[float(value) for value in data.qpos],
                site_positions=site_positions,
            )
        )
    return replay_frames


def save_urdf_replay_csv(
    output_csv_path: Path,
    replay_frames: list[URDFMuJoCoFrame],
    joint_names: list[str],
) -> None:
    """保存 URDF 仿真时序数据。"""
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)
    headers = ["step_index", "time_seconds"]
    headers.extend(f"target_{joint_name}" for joint_name in joint_names)
    headers.extend(f"qpos_{joint_name}" for joint_name in joint_names)

    site_names = sorted({site_name for frame in replay_frames for site_name in frame.site_positions})
    for site_name in site_names:
        headers.extend([f"{site_name}_x", f"{site_name}_y", f"{site_name}_z"])

    with output_csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(headers)
        for frame in replay_frames:
            row: list[float | int] = [frame.step_index, frame.time_seconds]
            row.extend(frame.joint_target)
            row.extend(frame.qpos)
            for site_name in site_names:
                row.extend(frame.site_positions.get(site_name, ["", "", ""]))
            writer.writerow(row)


def save_urdf_replay_plots(
    output_directory: Path,
    replay_frames: list[URDFMuJoCoFrame],
    joint_names: list[str],
    site_names: list[str],
    title_prefix: str,
) -> None:
    """保存关节跟踪图和末端轨迹图。"""
    _configure_matplotlib_fonts()
    output_directory.mkdir(parents=True, exist_ok=True)
    times = [frame.time_seconds for frame in replay_frames]

    figure, axes = plt.subplots(len(joint_names), 1, figsize=(12, 4 * len(joint_names)), sharex=True)
    if len(joint_names) == 1:
        axes = [axes]
    for joint_index, axis in enumerate(axes):
        target_values = [frame.joint_target[joint_index] for frame in replay_frames]
        qpos_values = [frame.qpos[joint_index] for frame in replay_frames]
        axis.plot(times, target_values, label="目标角度", linewidth=2.0)
        axis.plot(times, qpos_values, label="MuJoCo qpos", linestyle="--", linewidth=1.6)
        axis.set_ylabel(f"{joint_names[joint_index]}\n角度(rad)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("时间 (s)")
    figure.suptitle(f"{title_prefix} - 关节角度跟踪", fontsize=14)
    figure.tight_layout()
    figure.savefig(output_directory / "urdf_joint_tracking.png", dpi=160, bbox_inches="tight")
    plt.close(figure)

    available_sites = [site_name for site_name in site_names if any(site_name in frame.site_positions for frame in replay_frames)]
    if available_sites:
        figure = plt.figure(figsize=(7 * len(available_sites), 6))
        for subplot_index, site_name in enumerate(available_sites, start=1):
            axis = figure.add_subplot(1, len(available_sites), subplot_index, projection="3d")
            positions = [frame.site_positions[site_name] for frame in replay_frames if site_name in frame.site_positions]
            xs = [position[0] for position in positions]
            ys = [position[1] for position in positions]
            zs = [position[2] for position in positions]
            axis.plot(xs, ys, zs, linewidth=2.0)
            axis.scatter(xs[0], ys[0], zs[0], color="green", label="起点")
            axis.scatter(xs[-1], ys[-1], zs[-1], color="red", label="终点")
            axis.set_title(site_name)
            axis.set_xlabel("x")
            axis.set_ylabel("y")
            axis.set_zlabel("z")
            axis.legend(loc="best")
        figure.suptitle(f"{title_prefix} - 关键 site 轨迹", fontsize=14)
        figure.tight_layout()
        figure.savefig(output_directory / "urdf_site_trajectory.png", dpi=160, bbox_inches="tight")
        plt.close(figure)


def render_urdf_replay_gif(
    model: mujoco.MjModel,
    replay_frames: list[URDFMuJoCoFrame],
    output_gif_path: Path,
    width: int,
    height: int,
    render_fps: int,
    camera_lookat: list[float] | None = None,
    camera_distance: float | None = None,
    camera_azimuth: float | None = None,
    camera_elevation: float | None = None,
) -> bool:
    """离线渲染 URDF 简化仿真的 GIF。"""
    output_gif_path.parent.mkdir(parents=True, exist_ok=True)
    data = mujoco.MjData(model)
    apply_default_like_visual_preset(model)
    framebuffer_width = int(model.vis.global_.offwidth)
    framebuffer_height = int(model.vis.global_.offheight)
    renderer = mujoco.Renderer(
        model,
        width=min(int(width), framebuffer_width),
        height=min(int(height), framebuffer_height),
    )

    if len(replay_frames) >= 2:
        simulated_frequency = 1.0 / max(replay_frames[1].time_seconds - replay_frames[0].time_seconds, 1e-6)
    else:
        simulated_frequency = float(render_fps)
    frame_stride = max(1, int(round(simulated_frequency / max(render_fps, 1))))

    rendered_images = []
    for frame in replay_frames[::frame_stride]:
        data.qpos[:] = frame.qpos
        mujoco.mj_forward(model, data)
        if any(value is not None for value in [camera_lookat, camera_distance, camera_azimuth, camera_elevation]):
            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(camera)
            configure_free_camera(
                camera,
                lookat=camera_lookat,
                distance=camera_distance,
                azimuth=camera_azimuth,
                elevation=camera_elevation,
            )
            renderer.update_scene(data, camera=camera)
        else:
            renderer.update_scene(data)
        rendered_images.append(renderer.render())
    renderer.close()

    try:
        import imageio.v2 as imageio

        imageio.mimsave(output_gif_path, rendered_images, fps=render_fps)
        return True
    except ModuleNotFoundError:
        try:
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
        except ModuleNotFoundError:
            return False


def replay_urdf_in_viewer(
    model: mujoco.MjModel,
    replay_frames: list[URDFMuJoCoFrame],
    playback_speed: float,
    startup_pause_seconds: float = 0.8,
    keep_open_after_replay: bool = True,
    camera_lookat: list[float] | None = None,
    camera_distance: float | None = None,
    camera_azimuth: float | None = None,
    camera_elevation: float | None = None,
) -> None:
    """用 MuJoCo viewer 播放 URDF 教学仿真。"""
    import mujoco.viewer as viewer

    data = mujoco.MjData(model)
    apply_default_like_visual_preset(model)
    with viewer.launch_passive(model, data) as viewer_handle:
        if any(value is not None for value in [camera_lookat, camera_distance, camera_azimuth, camera_elevation]):
            configure_free_camera(
                viewer_handle.cam,
                lookat=camera_lookat,
                distance=camera_distance,
                azimuth=camera_azimuth,
                elevation=camera_elevation,
            )
        viewer_handle.sync()
        if startup_pause_seconds > 0:
            time.sleep(startup_pause_seconds)

        for frame in replay_frames:
            if not viewer_handle.is_running():
                return
            data.qpos[:] = frame.qpos
            mujoco.mj_forward(model, data)
            viewer_handle.sync()

            step_interval = replay_frames[1].time_seconds - replay_frames[0].time_seconds if len(replay_frames) >= 2 else 0.03
            time.sleep(step_interval / max(playback_speed, 1e-6))

        if keep_open_after_replay:
            while viewer_handle.is_running():
                viewer_handle.sync()
                time.sleep(0.03)


def load_urdf_demo_config(config_path: Path) -> dict[str, Any]:
    """读取 URDF MuJoCo 演示配置。"""
    return json.loads(config_path.read_text(encoding="utf-8"))
