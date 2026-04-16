# 13_URDF真实网格仿真说明

## 这次解决的是什么问题
之前的 `URDF 简化仿真` 用的是教学用几何体，能看懂结构和动作，但不是实际外观。  
这次新增的是“真实 STL 网格版”：

- 继续使用你给的 `urdf_0.1.4.2.urdf`
- 尽量保留真实外观
- 对超过 MuJoCo 面数限制的 STL 自动减面
- 生成 MuJoCo 可直接加载的运行时 URDF

## 为什么之前没直接成功
根因不是 URDF 结构，而是 STL 网格太重。

已经确认：
- `Link1.STL`：318,612 faces
- MuJoCo 直接导入单个 STL 的上限大约是 200,000 faces

所以现在新增了一步预处理：
- `Link1.STL` 减到 150,000 faces
- `Link2.STL` 减到 150,000 faces
- `base.STL` 和 `Link3.STL` 保持原样

## 真实网格资产会放到哪里
处理结果只会放在学习版目录，不会改原始模型：

- `SRL2_learning/assets/external_models/urdf_0.1.4.2/source/`
- `SRL2_learning/assets/external_models/urdf_0.1.4.2/processed_meshes/`
- `SRL2_learning/assets/external_models/urdf_0.1.4.2/runtime/`

含义：
- `source/`：原始 URDF 和 STL 的镜像副本
- `processed_meshes/`：减面后的 STL
- `runtime/`：MuJoCo 实际加载的运行时 URDF 和同目录 mesh

## 如何运行
先直接运行真实网格版：

```powershell
python SRL2_learning/scripts/run_urdf_real_mesh_demo.py
```

如果要打开 MuJoCo viewer：

```powershell
python SRL2_learning/scripts/run_urdf_real_mesh_demo.py --viewer
```

## 朝向和位置参数在哪里调
你要找的参数就在：
- [urdf_real_mesh_demo_config.json](/e:/project/codetest/SRL2_learning/configs/urdf_real_mesh_demo_config.json)

最关键的是这几个字段：

- `base_position`
  - 大白话：机械臂底座放在哪
  - 正式一点：`world -> base` 的平移，单位米
  - 例子：`[0.0, 0.0, 0.65]`

- `base_euler`
  - 大白话：机械臂底座朝哪边转
  - 正式一点：`world -> base` 的欧拉角，顺序是 `roll, pitch, yaw`，单位弧度
  - 例子：`[0.0, 0.0, -1.5708]` 表示绕 z 轴旋转约 `-90°`

- `viewer_camera_lookat`
  - 大白话：相机盯着哪里看

- `viewer_camera_distance`
  - 大白话：相机离模型多远

- `viewer_camera_azimuth`
  - 大白话：相机从水平方向绕着模型转到哪个角度

- `viewer_camera_elevation`
  - 大白话：相机从上往下还是从下往上看

真正把朝向写进运行时 URDF 的代码在：
- [urdf_real_mesh_mujoco.py](/e:/project/codetest/SRL2_learning/src/srl2_learning/adapters/urdf_real_mesh_mujoco.py)

关键位置是 `world_to_base` 这个固定关节。  
也就是说，真正决定“模型朝向”的不是 STL 文件本身，而是 `world -> base` 这一层变换。

如果只想先做预处理：

```powershell
python SRL2_learning/scripts/prepare_urdf_real_mesh.py --prepare-only
```

如果你改了网格处理策略，想强制重建：

```powershell
python SRL2_learning/scripts/run_urdf_real_mesh_demo.py --force-rebuild
```

## 运行后会输出什么
日志目录：
- `SRL2_learning/logs/urdf_real_mesh_demo/`

输出文件：
- `urdf_real_mesh_replay.csv`
- `urdf_joint_tracking.png`
- `urdf_site_trajectory.png`
- `urdf_real_mesh_replay.gif`

资产目录中还会额外生成：
- 运行时 URDF
- 预处理清单 `preprocess_manifest.json`

## 这版和简化版的关系
- 简化版：适合教学，结构清楚，依赖少
- 真实网格版：适合看实际外观，接近你想要的“导入真实模型”

建议顺序：
1. 先确认真实网格版能正常弹 viewer
2. 再微调 `base_position`、`base_euler`
3. 最后再考虑更复杂的动作或更多关节

## 当前边界
- 这依然不是原始 CAD 的无损直导入版本
- 为了满足 MuJoCo 限制，网格做了降面
- 目前仍然只处理你这套 3 关节 URDF，不扩展到原始 `SRL2` 的人体混合模型

## 为什么之前背景偏暗
不是因为 MuJoCo 不能显示默认场景，而是因为这条链路本质上是“URDF 导入”，不是直接手写完整 MJCF 场景。

结果就是：
- 模型本体能进 MuJoCo
- 但不会天然带一个你熟悉的、观感很亮的完整默认演示场景

这次已经做了两件调整：
- 把 viewer 和离线渲染的头灯参数调亮
- 给 free camera 一个更稳定的默认观察角度

所以现在的目标不是做一套花哨自定义场景，而是尽量接近 MuJoCo 默认 viewer 的观感，并优先保证你能清楚看见机械臂。
