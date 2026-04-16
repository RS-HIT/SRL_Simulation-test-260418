"""检查当前四坐标系定义和偏移常量。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.kinematics.forward_kinematics import compute_forward_kinematics
from srl2_learning.kinematics.model_context import load_real_mesh_model_context
from srl2_learning.kinematics.tool_frame import load_tool_frame_config


def main() -> None:
    experiment_config_path = PROJECT_ROOT / "configs" / "cartesian_target_experiment.json"
    experiment_config = json.loads(experiment_config_path.read_text(encoding="utf-8"))
    real_mesh_config_path = (experiment_config_path.parent / experiment_config["real_mesh_config_path"]).resolve()
    tool_frame_config_path = (experiment_config_path.parent / experiment_config["tool_frame_config_path"]).resolve()
    tool_frame = load_tool_frame_config(tool_frame_config_path)

    model_context = load_real_mesh_model_context(
        real_mesh_config_path=real_mesh_config_path,
        reference_body_name=experiment_config.get("flange_reference_body", experiment_config["reference_body_name"]),
        reference_site_name=experiment_config.get("flange_reference_site", experiment_config["reference_site_name"]),
    )
    fk_result = compute_forward_kinematics(
        context=model_context,
        joint_positions=experiment_config["initial_joint_positions"],
        tool_frame=tool_frame if experiment_config["use_tool_frame"] else None,
    )

    print("=== 四坐标系结构检查 ===")
    print(f"场景模型: {model_context.scene_model_path}")
    print("你现在最该改的常量位置：")
    print(f"- 基座偏移改这里: {real_mesh_config_path}")
    print(f"- 工具偏移改这里: {tool_frame_config_path}")
    print()
    print("joint 名称:")
    for name in model_context.joint_names:
        print(f"  - {name}")
    print("body 名称:")
    for name in model_context.body_names:
        print(f"  - {name}")
    print("site 名称:")
    for name in model_context.site_names:
        print(f"  - {name}")
    print()
    print("四个坐标系当前定义：")
    print(f"- world frame 原点: {fk_result.world_origin}")
    print(f"- base frame 原点: {fk_result.base_position}")
    print(f"- flange frame 参考类型: {fk_result.reference_kind}")
    print(f"- flange frame 参考名称: {fk_result.reference_name}")
    print(f"- flange frame 原点: {fk_result.flange_position}")
    print(f"- tool frame 原点: {fk_result.tool_position}")
    print(f"- tool 相对 flange 的世界系偏移: {fk_result.tool_offset_world}")
    print()
    print("当前配置来源：")
    print(f"- flange_reference_body: {experiment_config.get('flange_reference_body', experiment_config['reference_body_name'])}")
    print(f"- flange_reference_site: {experiment_config.get('flange_reference_site', experiment_config['reference_site_name'])}")
    print(f"- tool_frame_enabled: {tool_frame.enabled and experiment_config['use_tool_frame']}")
    print(f"- tool_translation_xyz: {tool_frame.tool_translation_xyz}")
    print(f"- tool_rotation_rpy: {tool_frame.tool_rotation_rpy}")
    print()
    print("误差比较点说明：")
    print("- 当前真正参与误差比较的是 tool frame 原点。")
    print("- 如果未启用工具偏移，则 tool frame 与 flange frame 重合。")


if __name__ == "__main__":
    main()
