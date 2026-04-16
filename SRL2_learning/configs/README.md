# 配置目录说明

当前目录用于存放学习版示例配置。

## `offline_demo_config.json`
这个配置服务于当前的最小离线教学示例，主要控制：
- 最小主链路的输入目标
- 教学用模拟实际关节响应
- CSV 和图表输出

适合的用途：
- 理解“目标 -> 规划 -> 状态更新 -> 映射 -> 输出”这条主链路
- 观察期望角度、模拟实际角度和误差之间的关系

## `mujoco_demo_config.json`
这个配置服务于原模型 MuJoCo 回放示例，主要控制四类信息。

1. 原模型来源
- `source_model_path`

2. 最小主链路动作输入
- `reference_model_path`
- `start_joint_command`
- `target_joint_command`
- `max_velocity`
- `max_acceleration`

3. MuJoCo 回放方式
- `arm_side`
- `playback_speed`
- `viewer_enabled`
- `offscreen_render_enabled`

4. 输出与观察项
- `render_width`
- `render_height`
- `render_fps`
- `output_directory_name`
- `observed_site_names`
- `dump_sanitized_xml_copy`

## 建议怎么改
如果你第一次试 MuJoCo 回放，优先改这几个参数：
- `target_joint_command`
- `playback_speed`
- `viewer_enabled`

如果你想更明显地看到末端运动轨迹，可以重点观察：
- `observed_site_names`

## 注意
- 当前 MuJoCo 配置仍然是教学用途，不等价原项目完整工程配置。
- `source_model_path` 虽然指向原 `Rsras1.xml`，但由于仓库缺失 STL，运行时会自动切到无网格结构回放模式。
