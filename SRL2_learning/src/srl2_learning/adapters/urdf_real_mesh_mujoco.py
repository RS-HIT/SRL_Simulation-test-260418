"""基于真实 STL 网格的 URDF -> MuJoCo 适配层。

这个文件在整条链路里的位置：
- 它是对 `urdf_mujoco_demo.py` 的补充，不替代原有简化教学模式。
- 简化模式解决“先把结构和动作跑起来”。
- 本文件解决“尽量保留真实外观，把实际 STL 导入 MuJoCo”。

核心思路：
- 原始 URDF 继续只读。
- 先把原 URDF 和 STL 镜像到 `SRL2_learning/assets/...`。
- 再对超过 MuJoCo 面数上限的 STL 做减面。
- 生成一个运行时 URDF，让 MuJoCo 直接加载真实网格版本。

当前边界：
- 这不是原始 CAD 外观的 100% 无损导入。
- 为了满足 MuJoCo 面数限制，允许做降面处理。
- 当前只处理你提供的这一套 3 关节 URDF。
"""

from __future__ import annotations

import importlib.util
import json
import shutil
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import mujoco
import trimesh


DEFAULT_REAL_MESH_FACE_LIMITS = {
    "base.STL": None,
    "Link1.STL": 150000,
    "Link2.STL": 150000,
    "Link3.STL": None,
}

# 这两个默认值就是“机械臂摆在世界坐标里的位置和朝向”。
# 新手先记一句大白话：
# - `base_position` 控制“机械臂底座放在哪”
# - `base_euler` 控制“机械臂底座朝哪边转”
#
# 正式一点的说法：
# - `base_position`: world -> base 的平移，单位米
# - `base_euler`: world -> base 的欧拉角，单位弧度，顺序是 roll / pitch / yaw
#
# 当前默认把 yaw 设成 -pi/2 左右，是为了让机械臂在默认 viewer 视角下更容易看到主体动作。
DEFAULT_REAL_MESH_BASE_POSITION = [0.0, 0.0, 0.65]
DEFAULT_REAL_MESH_BASE_EULER = [0.0, 0.0, -1.5708]
DEFAULT_REAL_MESH_ARENA_MEMORY = "64M"


@dataclass(slots=True)
class RealMeshProcessInfo:
    """单个 STL 的处理结果。"""

    mesh_name: str
    source_face_count: int
    output_face_count: int
    target_face_count: int | None
    was_decimated: bool
    source_path: Path
    processed_path: Path
    runtime_copy_path: Path


@dataclass(slots=True)
class RealMeshPreparationResult:
    """真实网格预处理结果。"""

    source_urdf_path: Path
    mirrored_urdf_path: Path
    runtime_urdf_path: Path
    runtime_mjcf_path: Path
    manifest_path: Path
    source_directory: Path
    processed_mesh_directory: Path
    runtime_directory: Path
    mesh_infos: list[RealMeshProcessInfo]


class URDFRealMeshPreparationAdapter:
    """负责把原始 URDF 预处理成 MuJoCo 可加载的真实网格版本。

    这个类为什么存在：
    - 真实模型导入的难点不在 URDF 结构，而在 STL 路径和网格复杂度。
    - 这个类把“镜像、减面、重写路径、生成运行时 URDF”收拢在一起。
    """

    def __init__(
        self,
        source_urdf_path: str | Path,
        processed_model_root: str | Path,
    ) -> None:
        self.source_urdf_path = Path(source_urdf_path).resolve()
        self.processed_model_root = Path(processed_model_root).resolve()
        self.source_directory = self.processed_model_root / "source"
        self.processed_mesh_directory = self.processed_model_root / "processed_meshes"
        self.runtime_directory = self.processed_model_root / "runtime"
        self.manifest_path = self.runtime_directory / "preprocess_manifest.json"

    def _ensure_directories(self) -> None:
        """创建预处理所需目录。"""
        self.source_directory.mkdir(parents=True, exist_ok=True)
        self.processed_mesh_directory.mkdir(parents=True, exist_ok=True)
        self.runtime_directory.mkdir(parents=True, exist_ok=True)

    def _source_mesh_directory(self) -> Path:
        """返回原始 STL 所在目录。"""
        return self.source_urdf_path.parent.parent / "meshes"

    def _copy_if_needed(self, source_path: Path, target_path: Path) -> None:
        """当目标不存在或内容变化时复制文件。"""
        if not target_path.exists():
            shutil.copy2(source_path, target_path)
            return
        if source_path.stat().st_size != target_path.stat().st_size or int(source_path.stat().st_mtime) != int(target_path.stat().st_mtime):
            shutil.copy2(source_path, target_path)

    def _mirror_source_assets(self) -> Path:
        """把原始 URDF 和 STL 镜像到学习版目录。"""
        mirrored_urdf_path = self.source_directory / self.source_urdf_path.name
        self._copy_if_needed(self.source_urdf_path, mirrored_urdf_path)
        for mesh_path in self._source_mesh_directory().glob("*.STL"):
            self._copy_if_needed(mesh_path, self.source_directory / mesh_path.name)
        return mirrored_urdf_path

    def _load_mesh_face_count(self, mesh_path: Path) -> int:
        """读取 STL 面数。"""
        mesh = trimesh.load_mesh(mesh_path, force="mesh")
        return int(len(mesh.faces))

    def _process_mesh(self, mesh_name: str, target_face_count: int | None) -> RealMeshProcessInfo:
        """必要时对单个 STL 做减面。

        什么时候调用它：
        - 在生成运行时 URDF 之前调用。
        - 因为运行时 URDF 里的 mesh 一旦超过 MuJoCo 上限，整个模型会直接加载失败。
        """
        source_path = self.source_directory / mesh_name
        processed_path = self.processed_mesh_directory / mesh_name
        runtime_copy_path = self.runtime_directory / mesh_name

        source_mesh = trimesh.load_mesh(source_path, force="mesh")
        source_face_count = int(len(source_mesh.faces))
        working_mesh = source_mesh
        was_decimated = False

        if target_face_count is not None and source_face_count > target_face_count:
            try:
                working_mesh = source_mesh.simplify_quadric_decimation(face_count=target_face_count)
            except ModuleNotFoundError as exc:
                raise ModuleNotFoundError(
                    "当前环境缺少 fast_simplification，无法重新减面。"
                    "如果你只是修改了 base_euler、base_position 或相机参数，请直接复用已有 processed_meshes；"
                    "如果你确实需要重新减面，请在当前环境执行：pip install fast-simplification"
                ) from exc
            was_decimated = True

        output_face_count = int(len(working_mesh.faces))
        working_mesh.export(processed_path)
        shutil.copy2(processed_path, runtime_copy_path)

        return RealMeshProcessInfo(
            mesh_name=mesh_name,
            source_face_count=source_face_count,
            output_face_count=output_face_count,
            target_face_count=target_face_count,
            was_decimated=was_decimated,
            source_path=source_path,
            processed_path=processed_path,
            runtime_copy_path=runtime_copy_path,
        )

    def _manifest_payload(
        self,
        mesh_face_limits: dict[str, int | None],
        base_position: list[float],
        base_euler: list[float],
        ground_height: float,
        add_ground_plane: bool,
        default_joint_limit: tuple[float, float],
        mesh_infos: list[RealMeshProcessInfo],
    ) -> dict[str, Any]:
        """生成预处理清单。"""
        source_mesh_states = {}
        for mesh_path in self._source_mesh_directory().glob("*.STL"):
            source_mesh_states[mesh_path.name] = {
                "size": mesh_path.stat().st_size,
                "mtime": int(mesh_path.stat().st_mtime),
            }

        return {
            "version": 1,
            "source_urdf_path": str(self.source_urdf_path),
            "source_urdf_size": self.source_urdf_path.stat().st_size,
            "source_urdf_mtime": int(self.source_urdf_path.stat().st_mtime),
            "mesh_face_limits": mesh_face_limits,
            "base_position": base_position,
            "base_euler": base_euler,
            "ground_height": ground_height,
            "add_ground_plane": add_ground_plane,
            "default_joint_limit": list(default_joint_limit),
            "source_mesh_states": source_mesh_states,
            "mesh_infos": [
                {
                    "mesh_name": info.mesh_name,
                    "source_face_count": info.source_face_count,
                    "output_face_count": info.output_face_count,
                    "target_face_count": info.target_face_count,
                    "was_decimated": info.was_decimated,
                }
                for info in mesh_infos
            ],
        }

    def _runtime_outputs_exist(self, mesh_face_limits: dict[str, int | None]) -> bool:
        """检查预处理结果是否完整。"""
        runtime_urdf_path = self.runtime_directory / self.source_urdf_path.name
        if not runtime_urdf_path.exists() or not self.manifest_path.exists():
            return False
        for mesh_name in mesh_face_limits:
            if not (self.processed_mesh_directory / mesh_name).exists():
                return False
            if not (self.runtime_directory / mesh_name).exists():
                return False
        return True

    def _source_mesh_states_match_manifest(self, manifest: dict[str, Any]) -> bool:
        """检查原始 STL 是否仍与清单一致。"""
        for mesh_path in self._source_mesh_directory().glob("*.STL"):
            state = manifest.get("source_mesh_states", {}).get(mesh_path.name)
            if state is None:
                return False
            if state.get("size") != mesh_path.stat().st_size or state.get("mtime") != int(mesh_path.stat().st_mtime):
                return False
        return True

    def _can_reuse_manifest(
        self,
        mesh_face_limits: dict[str, int | None],
        base_position: list[float],
        base_euler: list[float],
        ground_height: float,
        add_ground_plane: bool,
        default_joint_limit: tuple[float, float],
    ) -> bool:
        """判断是否可以直接复用已有预处理结果。"""
        if not self._runtime_outputs_exist(mesh_face_limits):
            return False
        try:
            manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        except Exception:
            return False

        expected = {
            "version": 1,
            "source_urdf_path": str(self.source_urdf_path),
            "source_urdf_size": self.source_urdf_path.stat().st_size,
            "source_urdf_mtime": int(self.source_urdf_path.stat().st_mtime),
            "mesh_face_limits": mesh_face_limits,
            "base_position": base_position,
            "base_euler": base_euler,
            "ground_height": ground_height,
            "add_ground_plane": add_ground_plane,
            "default_joint_limit": list(default_joint_limit),
        }
        for key, value in expected.items():
            if manifest.get(key) != value:
                return False
        return self._source_mesh_states_match_manifest(manifest)

    def _can_reuse_processed_meshes(
        self,
        mesh_face_limits: dict[str, int | None],
    ) -> bool:
        """判断是否可以复用已有减面结果而只重写运行时 URDF。

        这个判断故意不看 base_position / base_euler / ground_height。
        因为这些只是展示参数，改它们不应该强迫用户重新减面。
        """
        if not self._runtime_outputs_exist(mesh_face_limits):
            return False
        try:
            manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        except Exception:
            return False
        expected = {
            "version": 1,
            "source_urdf_path": str(self.source_urdf_path),
            "source_urdf_size": self.source_urdf_path.stat().st_size,
            "source_urdf_mtime": int(self.source_urdf_path.stat().st_mtime),
            "mesh_face_limits": mesh_face_limits,
        }
        for key, value in expected.items():
            if manifest.get(key) != value:
                return False
        return self._source_mesh_states_match_manifest(manifest)

    def _reuse_existing_processed_mesh(self, mesh_name: str, target_face_count: int | None) -> RealMeshProcessInfo:
        """复用已有 processed mesh，而不是重新减面。"""
        source_path = self.source_directory / mesh_name
        processed_path = self.processed_mesh_directory / mesh_name
        runtime_copy_path = self.runtime_directory / mesh_name
        if not processed_path.exists():
            raise FileNotFoundError(f"找不到已处理网格：{processed_path}")
        shutil.copy2(processed_path, runtime_copy_path)
        source_face_count = self._load_mesh_face_count(source_path)
        output_face_count = self._load_mesh_face_count(processed_path)
        return RealMeshProcessInfo(
            mesh_name=mesh_name,
            source_face_count=source_face_count,
            output_face_count=output_face_count,
            target_face_count=target_face_count,
            was_decimated=output_face_count < source_face_count,
            source_path=source_path,
            processed_path=processed_path,
            runtime_copy_path=runtime_copy_path,
        )

    def _build_runtime_urdf(
        self,
        mirrored_urdf_path: Path,
        base_position: list[float],
        base_euler: list[float],
        ground_height: float,
        add_ground_plane: bool,
        default_joint_limit: tuple[float, float],
        mesh_face_limits: dict[str, int | None],
        reuse_processed_meshes: bool = False,
    ) -> tuple[Path, list[RealMeshProcessInfo]]:
        """生成 MuJoCo 可直接加载的运行时 URDF。"""
        root = ET.parse(mirrored_urdf_path).getroot()
        processed_cache: dict[str, RealMeshProcessInfo] = {}

        world_link = ET.Element("link", name="world")

        # 这就是“朝向参数真正生效”的地方。
        # MuJoCo 最终看到的机械臂姿态，不是由 STL 自己决定的，
        # 而是由这里这个 world -> base 的固定关节决定的。
        #
        # 其中：
        # - `base_position` 决定底座在世界坐标中的位置
        # - `base_euler` 决定底座相对世界坐标的旋转
        #
        # 如果你后面还想继续调朝向，优先改配置文件里的：
        # - `base_position`
        # - `base_euler`
        world_to_base_joint = ET.Element("joint", name="world_to_base", type="fixed")
        ET.SubElement(world_to_base_joint, "origin", xyz=" ".join(f"{value:.6f}" for value in base_position), rpy=" ".join(f"{value:.6f}" for value in base_euler))
        ET.SubElement(world_to_base_joint, "parent", link="world")
        ET.SubElement(world_to_base_joint, "child", link="base")

        root.insert(0, world_link)
        root.insert(1, world_to_base_joint)

        if add_ground_plane:
            ground_link = ET.Element("link", name="ground")
            visual = ET.SubElement(ground_link, "visual")
            ET.SubElement(
                visual,
                "origin",
                xyz=f"0 0 {float(ground_height) - 0.05:.6f}",
                rpy="0 0 0",
            )
            visual_geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(visual_geometry, "box", size="4 4 0.1")
            material = ET.SubElement(visual, "material", name="ground_mat")
            ET.SubElement(material, "color", rgba="0.80 0.80 0.80 1")

            collision = ET.SubElement(ground_link, "collision")
            ET.SubElement(
                collision,
                "origin",
                xyz=f"0 0 {float(ground_height) - 0.05:.6f}",
                rpy="0 0 0",
            )
            collision_geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(collision_geometry, "box", size="4 4 0.1")

            world_to_ground_joint = ET.Element("joint", name="world_to_ground", type="fixed")
            ET.SubElement(world_to_ground_joint, "origin", xyz="0 0 0", rpy="0 0 0")
            ET.SubElement(world_to_ground_joint, "parent", link="world")
            ET.SubElement(world_to_ground_joint, "child", link="ground")

            root.insert(1, ground_link)
            root.insert(3, world_to_ground_joint)

        for joint_element in root.findall("joint"):
            if joint_element.get("type") != "revolute":
                continue
            limit_element = joint_element.find("limit")
            if limit_element is None:
                continue
            lower_limit = float(limit_element.get("lower", default_joint_limit[0]))
            upper_limit = float(limit_element.get("upper", default_joint_limit[1]))
            if abs(lower_limit - upper_limit) < 1e-9:
                lower_limit, upper_limit = default_joint_limit
            limit_element.set("lower", f"{lower_limit:.6f}")
            limit_element.set("upper", f"{upper_limit:.6f}")
            limit_element.set("effort", limit_element.get("effort", "10") if float(limit_element.get("effort", "0")) > 0 else "10")
            limit_element.set("velocity", limit_element.get("velocity", "2") if float(limit_element.get("velocity", "0")) > 0 else "2")

        mesh_infos: list[RealMeshProcessInfo] = []
        for mesh_element in root.findall(".//mesh"):
            filename = mesh_element.get("filename")
            if not filename:
                continue
            mesh_name = Path(filename).name
            if mesh_name not in mesh_face_limits:
                continue
            if mesh_name not in processed_cache:
                if reuse_processed_meshes:
                    processed_cache[mesh_name] = self._reuse_existing_processed_mesh(mesh_name, mesh_face_limits[mesh_name])
                else:
                    processed_cache[mesh_name] = self._process_mesh(mesh_name, mesh_face_limits[mesh_name])
                mesh_infos.append(processed_cache[mesh_name])
            # MuJoCo 读取 URDF 时会在 URDF 所在目录里找 mesh basename。
            # 所以这里不能保留 package:// 或复杂相对路径，而要直接写文件名，
            # 并保证同名 STL 已经复制到 runtime 目录。
            mesh_element.set("filename", mesh_name)

        runtime_urdf_path = self.runtime_directory / self.source_urdf_path.name
        ET.ElementTree(root).write(runtime_urdf_path, encoding="utf-8", xml_declaration=True)
        return runtime_urdf_path, mesh_infos

    def _inject_default_like_scene_into_mjcf(self, mjcf_path: Path) -> None:
        """把编译后的 MJCF 调成更接近 MuJoCo 默认 viewer 的场景。

        重点不是做花哨场景，而是补上 URDF 直导入天然缺失的 skybox/visual 配置。
        黑背景的根因基本就在这里。
        """
        root = ET.parse(mjcf_path).getroot()
        size = root.find("size")
        if size is None:
            size = ET.SubElement(root, "size")
        # 真实网格、地板和接触一起出现时，MuJoCo 很容易在首帧就生成大量约束。
        # arena memory 太小时，viewer 会报 “Insufficient arena memory ...”
        # 并可能让调试类脚本在窗口刚弹出后就异常退出。
        size.set("memory", DEFAULT_REAL_MESH_ARENA_MEMORY)

        asset = root.find("asset")
        if asset is None:
            asset = ET.SubElement(root, "asset")

        has_skybox = any(
            texture_element.get("type") == "skybox"
            for texture_element in asset.findall("texture")
        )
        if not has_skybox:
            ET.SubElement(
                asset,
                "texture",
                name="default_skybox",
                type="skybox",
                builtin="gradient",
                rgb1="0.30 0.45 0.70",
                rgb2="0.98 0.98 1.00",
                width="512",
                height="3072",
            )

        has_ground_texture = any(
            texture_element.get("name") == "groundplane_texture"
            for texture_element in asset.findall("texture")
        )
        if not has_ground_texture:
            ET.SubElement(
                asset,
                "texture",
                name="groundplane_texture",
                type="2d",
                builtin="checker",
                rgb1="0.20 0.30 0.40",
                rgb2="0.85 0.88 0.92",
                width="512",
                height="512",
            )

        has_ground_material = any(
            material_element.get("name") == "groundplane_material"
            for material_element in asset.findall("material")
        )
        if not has_ground_material:
            ET.SubElement(
                asset,
                "material",
                name="groundplane_material",
                texture="groundplane_texture",
                texrepeat="4 4",
                texuniform="true",
                reflectance="0.2",
            )

        visual = root.find("visual")
        if visual is None:
            visual = ET.SubElement(root, "visual")

        headlight = visual.find("headlight")
        if headlight is None:
            headlight = ET.SubElement(visual, "headlight")
        headlight.set("ambient", "0.45 0.45 0.45")
        headlight.set("diffuse", "0.85 0.85 0.85")
        headlight.set("specular", "0.25 0.25 0.25")

        rgba = visual.find("rgba")
        if rgba is None:
            rgba = ET.SubElement(visual, "rgba")
        rgba.set("haze", "0.92 0.95 1.0 1")

        global_visual = visual.find("global")
        if global_visual is None:
            global_visual = ET.SubElement(visual, "global")
        global_visual.set("offwidth", "1280")
        global_visual.set("offheight", "960")
        global_visual.set("azimuth", "120")
        global_visual.set("elevation", "-18")

        worldbody = root.find("worldbody")
        if worldbody is None:
            worldbody = ET.SubElement(root, "worldbody")

        has_floor = any(
            geom_element.get("name") == "scene_floor"
            for geom_element in worldbody.findall("geom")
        )
        if not has_floor:
            worldbody.insert(
                0,
                ET.Element(
                    "geom",
                    name="scene_floor",
                    type="plane",
                    size="6 6 0.1",
                    pos="0 0 0",
                    material="groundplane_material",
                ),
            )

        has_light = any(
            light_element.get("name") == "scene_light"
            for light_element in worldbody.findall("light")
        )
        if not has_light:
            worldbody.insert(
                0,
                ET.Element(
                    "light",
                    name="scene_light",
                    pos="0 0 4",
                    dir="0 0 -1",
                    diffuse="0.9 0.9 0.9",
                    specular="0.25 0.25 0.25",
                    directional="true",
                ),
            )

        ET.ElementTree(root).write(mjcf_path, encoding="utf-8", xml_declaration=True)

    def _build_runtime_mjcf(self, runtime_urdf_path: Path) -> Path:
        """把运行时 URDF 编译成 MJCF，并补默认风格场景。"""
        runtime_mjcf_path = self.runtime_directory / f"{self.source_urdf_path.stem}_runtime_scene.xml"
        model = mujoco.MjModel.from_xml_path(str(runtime_urdf_path))
        mujoco.mj_saveLastXML(str(runtime_mjcf_path), model)
        self._inject_default_like_scene_into_mjcf(runtime_mjcf_path)
        return runtime_mjcf_path

    def prepare_assets(
        self,
        mesh_face_limits: dict[str, int | None] | None = None,
        base_position: list[float] | None = None,
        base_euler: list[float] | None = None,
        ground_height: float = 0.0,
        add_ground_plane: bool = False,
        default_joint_limit: tuple[float, float] = (-3.14159, 3.14159),
        force_rebuild: bool = False,
    ) -> RealMeshPreparationResult:
        """预处理真实网格资产。

        如果结果已存在且配置未变，会直接复用。
        """
        self._ensure_directories()
        mesh_face_limits = dict(DEFAULT_REAL_MESH_FACE_LIMITS if mesh_face_limits is None else mesh_face_limits)
        base_position = DEFAULT_REAL_MESH_BASE_POSITION.copy() if base_position is None else [float(value) for value in base_position]
        base_euler = DEFAULT_REAL_MESH_BASE_EULER.copy() if base_euler is None else [float(value) for value in base_euler]

        mirrored_urdf_path = self._mirror_source_assets()
        runtime_urdf_path = self.runtime_directory / self.source_urdf_path.name

        if not force_rebuild and self._can_reuse_manifest(
            mesh_face_limits=mesh_face_limits,
            base_position=base_position,
            base_euler=base_euler,
            ground_height=ground_height,
            add_ground_plane=add_ground_plane,
            default_joint_limit=default_joint_limit,
        ):
            manifest = json.loads(self.manifest_path.read_text(encoding="utf-8"))
            runtime_mjcf_path = self.runtime_directory / f"{self.source_urdf_path.stem}_runtime_scene.xml"
            if not runtime_mjcf_path.exists():
                runtime_mjcf_path = self._build_runtime_mjcf(runtime_urdf_path)
            else:
                self._inject_default_like_scene_into_mjcf(runtime_mjcf_path)
            mesh_infos = [
                RealMeshProcessInfo(
                    mesh_name=item["mesh_name"],
                    source_face_count=item["source_face_count"],
                    output_face_count=item["output_face_count"],
                    target_face_count=item["target_face_count"],
                    was_decimated=item["was_decimated"],
                    source_path=self.source_directory / item["mesh_name"],
                    processed_path=self.processed_mesh_directory / item["mesh_name"],
                    runtime_copy_path=self.runtime_directory / item["mesh_name"],
                )
                for item in manifest.get("mesh_infos", [])
            ]
            return RealMeshPreparationResult(
                source_urdf_path=self.source_urdf_path,
                mirrored_urdf_path=mirrored_urdf_path,
                runtime_urdf_path=runtime_urdf_path,
                runtime_mjcf_path=runtime_mjcf_path,
                manifest_path=self.manifest_path,
                source_directory=self.source_directory,
                processed_mesh_directory=self.processed_mesh_directory,
                runtime_directory=self.runtime_directory,
                mesh_infos=mesh_infos,
            )

        reuse_processed_meshes = (
            not force_rebuild
            and self._can_reuse_processed_meshes(mesh_face_limits=mesh_face_limits)
        )
        runtime_urdf_path, mesh_infos = self._build_runtime_urdf(
            mirrored_urdf_path=mirrored_urdf_path,
            base_position=base_position,
            base_euler=base_euler,
            ground_height=ground_height,
            add_ground_plane=add_ground_plane,
            default_joint_limit=default_joint_limit,
            mesh_face_limits=mesh_face_limits,
            reuse_processed_meshes=reuse_processed_meshes,
        )
        runtime_mjcf_path = self._build_runtime_mjcf(runtime_urdf_path)
        manifest_payload = self._manifest_payload(
            mesh_face_limits=mesh_face_limits,
            base_position=base_position,
            base_euler=base_euler,
            ground_height=ground_height,
            add_ground_plane=add_ground_plane,
            default_joint_limit=default_joint_limit,
            mesh_infos=mesh_infos,
        )
        self.manifest_path.write_text(json.dumps(manifest_payload, indent=2, ensure_ascii=False), encoding="utf-8")

        return RealMeshPreparationResult(
            source_urdf_path=self.source_urdf_path,
            mirrored_urdf_path=mirrored_urdf_path,
            runtime_urdf_path=runtime_urdf_path,
            runtime_mjcf_path=runtime_mjcf_path,
            manifest_path=self.manifest_path,
            source_directory=self.source_directory,
            processed_mesh_directory=self.processed_mesh_directory,
            runtime_directory=self.runtime_directory,
            mesh_infos=mesh_infos,
        )


def load_real_mesh_mujoco_model(runtime_urdf_path: str | Path) -> mujoco.MjModel:
    """加载预处理后的真实网格 URDF。"""
    model = mujoco.MjModel.from_xml_path(str(Path(runtime_urdf_path).resolve()))
    return model


def load_real_mesh_demo_config(config_path: Path) -> dict[str, Any]:
    """读取真实网格演示配置。"""
    return json.loads(config_path.read_text(encoding="utf-8"))
