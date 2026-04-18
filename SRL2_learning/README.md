# SRL2_learning

`SRL2_learning` 是基于原始参考工程 `SRL2` 制作的教学友好版实验项目。

这个目录只在 `codetest/SRL2_learning` 内施工：
- 原始参考目录：`codetest/SRL2`
- 学习版目录：`codetest/SRL2_learning`
- `SRL2` 始终只读，不允许修改

## 当前重点

当前这版最适合你直接上手的是“末端坐标驱动实验”。

它做的事情很直接：
1. 你给一个世界坐标系下的目标点 `(x, y, z)`
2. 程序用教学版数值逆运动学求一组关节角
3. 再生成关节轨迹
4. 用 MuJoCo 回放动作
5. 记录目标点、法兰点、工具点和误差

## 四个坐标系先讲清楚

这版项目固定使用四个坐标系：

1. `world frame`
   整个仿真世界的大地坐标系。它固定不动。

2. `base frame`
   机械臂基座坐标系。它表示“机械臂安装在世界里的位置和朝向”。

3. `flange frame`
   末端法兰坐标系。它表示“机械臂本体最后一级参考点”。

4. `tool frame`
   工具坐标系。它表示“真正拿来比较误差和执行任务的点”。

当前关系固定为：

`world -> base -> flange -> tool`

其中：
- `base frame` 来自运行时 URDF 里的 `world_to_base` 固定变换
- `flange frame` 默认参考 `Link3 body`
- `tool frame = flange frame + 工具偏移`

## 当前误差到底基于哪个点

当前默认误差比较点是：

- `tool frame` 原点

这点非常重要。

也就是说：
- 如果你没有启用工具偏移，那么 `tool frame` 与 `flange frame` 重合
- 如果你启用了工具偏移，那么误差比较的是“法兰点加偏移后的工具点”，不是单纯 `Link3` 的 body 原点

## 你最常改的两个常量位置

### 1. 基座坐标系常量

如果你想改“机械臂底座在世界里的位置和朝向”，改这里：

- [urdf_real_mesh_demo_config.json](/e:/project/codetest/SRL2_learning/configs/urdf_real_mesh_demo_config.json)

关键字段：
- `base_position`
- `base_euler`

### 2. 工具坐标系常量

如果你想改“工具点相对法兰点的偏移”，改这里：

- [tool_frame_config.json](/e:/project/codetest/SRL2_learning/configs/tool_frame_config.json)

关键字段：
- `tool_translation_xyz`
- `tool_rotation_rpy`

兼容说明：
- 旧字段 `translation_xyz` / `rotation_rpy` 仍然能读
- 新代码优先使用 `tool_translation_xyz` / `tool_rotation_rpy`

## viewer 默认行为

现在运行末端实验时，viewer 默认开启。

最小运行命令：

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py
```

默认会看到四组坐标系：
- world frame
- base frame
- flange frame
- tool frame

并且每组坐标系原点旁边会显示名字：
- `world`
- `base`
- `flange`
- `tool`

颜色固定为：
- X 轴红色
- Y 轴绿色
- Z 轴蓝色

如果你想关闭 viewer：

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py --no-viewer
```

## 如何检查“误差为什么看起来很小”

优先做这三件事：

1. 运行结构检查脚本

```powershell
python SRL2_learning/scripts/inspect_end_effector_frames.py
```

它会打印：
- 当前场景模型路径
- world/base/flange/tool 四个坐标系当前定义
- 当前法兰参考来自哪个 body 或 site
- 当前工具偏移是多少
- 当前真正参与误差比较的是哪个点

2. 看 viewer 里的四组坐标轴

重点看：
- `flange frame` 在哪
- `tool frame` 是否真的相对 `flange frame` 发生了偏移

3. 看 CSV

当前 `cartesian_tracking.csv` 里会明确导出：
- `base_x/base_y/base_z`
- `flange_x/flange_y/flange_z`
- `tool_x/tool_y/tool_z`
- `target_x/target_y/target_z`
- `tool_error_x/tool_error_y/tool_error_z`
- `tool_error_norm`

推荐优先看 `tool_*` 字段，因为当前误差比较就是基于工具点。

## 末端坐标驱动实验

### 默认单目标实验

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py
```

### 指定目标点

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py --target-x 0.30 --target-y 0.05 --target-z 0.92
```

### 多目标实验

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py --experiment-mode multi
```

### 工具偏移敏感性实验

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py --experiment-mode tcp_sensitivity
```

说明：
- 命令行参数为了兼容旧版本，模式名仍保留 `tcp_sensitivity`
- 但现在文档里推荐你把它理解为“tool frame 偏移敏感性实验”

## 输出文件

末端实验通常会输出：
- `cartesian_tracking.csv`
- `cartesian_target_summary.csv`
- `cartesian_position_tracking.png`
- `cartesian_error.png`
- `cartesian_trajectory_xy.png`
- `cartesian_trajectory_3d.png`

## 哪些结果是严格读取，哪些是教学近似

### 严格来自 MuJoCo 位姿读取

- 给定关节角后的 `flange frame` 世界坐标
- 给定关节角后的 `tool frame` 世界坐标
- viewer 里显示的四组 frame 原点和方向

### 教学版近似

- 逆运动学是教学版数值法，不是工业级 IK 求解器
- 当前没有专用工业 TCP site
- 当前默认法兰参考仍然只是 `Link3 body`
- 当前主要做位置目标，不做工业级姿态约束

## 相关文档

- [14_末端坐标驱动实验说明.md](/e:/project/codetest/SRL2_learning/docs/14_末端坐标驱动实验说明.md)
- [13_URDF真实网格仿真说明.md](/e:/project/codetest/SRL2_learning/docs/13_URDF真实网格仿真说明.md)

## Viewer 初始视角怎么调

如果你觉得 MuJoCo viewer 一打开时的朝向和位置不合适，现在统一改这里：

- [cartesian_target_experiment.json](/e:/project/codetest/SRL2_learning/configs/cartesian_target_experiment.json)

关键字段：

- `viewer_camera_lookat`
- `viewer_camera_distance`
- `viewer_camera_azimuth`
- `viewer_camera_elevation`

这些字段同时会影响：

- [debug_joint_angle_tuner.py](/e:/project/codetest/SRL2_learning/scripts/debug_joint_angle_tuner.py)
- [run_cartesian_target_experiment.py](/e:/project/codetest/SRL2_learning/scripts/run_cartesian_target_experiment.py)

更详细的调整说明看：

- [18_关节角拖动调试器说明.md](/e:/project/codetest/SRL2_learning/docs/18_关节角拖动调试器说明.md)
