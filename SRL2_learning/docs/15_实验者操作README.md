# 15_实验者操作README

这份说明只回答实验时最常见的几个问题：

1. 世界坐标系下的目标点在哪里设置
2. 逆运动学解的过程在哪里体现
3. 工具坐标系原点和期望坐标的差值在哪里记录
4. 实验者应该按什么顺序修改配置、运行脚本、查看结果

## 1. 世界坐标系的点在哪里设置

这里要先分清两个概念。

### 1.1 世界坐标系原点

这是 viewer 里 `world frame` 画出来的位置，不是机械臂要去的目标点。

配置文件：

- [cartesian_target_experiment.json](/e:/project/codetest/SRL2_learning/configs/cartesian_target_experiment.json)

字段：

```json
"world_frame_origin": [0.0, 0.0, 0.0]
```

### 1.2 世界坐标系下的目标点

这是逆运动学真正要追的目标位置。

同样在这个配置文件里：

- [cartesian_target_experiment.json](/e:/project/codetest/SRL2_learning/configs/cartesian_target_experiment.json)

单目标实验字段：

```json
"target_position": [0.106287, -0.41627, 1.132398]
```

多目标实验字段：

```json
"target_points": [
  [0.106287, -0.41627, 1.132398],
  [0.186802, -0.441837, 1.064071]
]
```

也可以直接用命令行覆盖：

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py --target-x 0.30 --target-y 0.05 --target-z 0.92
```

## 2. 逆运动学解的过程在哪里体现

### 2.1 实验主流程里在哪里触发

文件：

- [cartesian_target_experiment.py](/e:/project/codetest/SRL2_learning/src/srl2_learning/experiments/cartesian_target_experiment.py)

关键函数：

- `run_single_target_experiment(...)`
- `run_multi_target_experiment(...)`

在这两个函数里会调用：

```python
ik_result = solve_inverse_kinematics(...)
```

这一步的输入是目标点，输出是一组目标关节角。

### 2.2 真正的 IK 算法在哪里

文件：

- [inverse_kinematics.py](/e:/project/codetest/SRL2_learning/src/srl2_learning/kinematics/inverse_kinematics.py)

当前方法：

- `Damped Least Squares`
- 位置误差驱动
- Jacobian 用有限差分近似

关键流程是：

1. 先做正运动学，得到当前工具点位置
2. 计算误差

```python
error_vector = target - current_position
```

3. 计算位置 Jacobian
4. 解出关节增量

```python
delta_joint = step_size * solve((J^T J + λ^2 I), J^T error)
```

5. 更新关节角并继续迭代
6. 收敛就返回；不收敛也返回当前最优结果

## 3. tool 坐标系原点和期望坐标的差值在哪里记录

### 3.1 在代码里怎么计算

文件：

- [cartesian_target_experiment.py](/e:/project/codetest/SRL2_learning/src/srl2_learning/experiments/cartesian_target_experiment.py)

关键函数：

- `_sample_from_fk_result(...)`

这里会直接计算：

```python
target_array = np.asarray(target_position, dtype=float)
tool_array = np.asarray(fk_result.tool_position, dtype=float)
tool_error = target_array - tool_array
```

然后保存到：

- `tool_error_xyz`
- `tool_error_norm`

也就是说，当前误差定义就是：

```text
工具误差 = 目标点 - 实际工具点
```

### 3.2 在结果文件里怎么看

文件由这里导出：

- [cartesian_visualization.py](/e:/project/codetest/SRL2_learning/src/srl2_learning/adapters/cartesian_visualization.py)

重点结果文件：

- `cartesian_tracking.csv`

你重点看这些列：

- `target_x`
- `target_y`
- `target_z`
- `tool_x`
- `tool_y`
- `tool_z`
- `tool_error_x`
- `tool_error_y`
- `tool_error_z`
- `tool_error_norm`

## 4. 实验者怎么做实验

推荐顺序如下。

### 第一步：先看四坐标系当前定义

运行：

```powershell
python SRL2_learning/scripts/inspect_end_effector_frames.py
```

这个脚本会打印：

- world frame 原点
- base frame 原点
- flange frame 参考是谁
- tool frame 原点
- 当前误差比较点是谁

### 第二步：修改配置

#### 改目标点

文件：

- [cartesian_target_experiment.json](/e:/project/codetest/SRL2_learning/configs/cartesian_target_experiment.json)

改：

- `target_position`
- 或 `target_points`

如果你希望在 viewer 里直接看到目标点位置，当前也在同一个配置里控制：

- `show_target_point`
- `show_frame_names`
- `target_marker_radius`

含义：
- `show_target_point = true` 时，会把目标点画成紫红色球 marker
- 单目标实验会显示 `target_1`
- 多目标实验会同时显示 `target_1 / target_2 / ...`
- `show_frame_names = true` 时，目标点旁边也会显示名字
- `target_marker_radius` 控制目标点球的大小

#### 改基座偏移

文件：

- [urdf_real_mesh_demo_config.json](/e:/project/codetest/SRL2_learning/configs/urdf_real_mesh_demo_config.json)

改：

- `base_position`
- `base_euler`

#### 改工具偏移

文件：

- [tool_frame_config.json](/e:/project/codetest/SRL2_learning/configs/tool_frame_config.json)

改：

- `tool_translation_xyz`
- `tool_rotation_rpy`

### 第三步：运行实验

默认运行：

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py
```

无窗口运行：

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py --no-viewer
```

### 多目标点实验怎么做

#### 方法一：直接改配置文件

在 [cartesian_target_experiment.json](/e:/project/codetest/SRL2_learning/configs/cartesian_target_experiment.json) 里：

1. 把

```json
"experiment_mode": "single"
```

改成：

```json
"experiment_mode": "multi"
```

2. 修改：

```json
"target_points": [
  [0.106287, -0.41627, 1.132398],
  [0.186802, -0.441837, 1.064071],
  [0.056339, -0.561542, 0.797476]
]
```

这里每一行就是一个世界坐标系下的目标点。

3. 运行：

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py
```

#### 方法二：命令行切到多目标模式

如果配置文件里已经写好了 `target_points`，可以直接运行：

```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py --experiment-mode multi
```

#### 多目标实验时 viewer 会看到什么

- 机械臂会按 `target_points` 的顺序逐个运动
- 每个目标点在 viewer 中都会显示成一个目标点 marker
- 默认名字是：
  - `target_1`
  - `target_2`
  - `target_3`
- 机械臂到达一个点后会按 `hold_steps` 保持若干步，再去下一个点

#### 多目标实验看什么结果

重点看：

- `cartesian_target_summary.csv`

这个文件会汇总每个目标点的最终结果，包括：
- 每个目标点的最终 tool 位置
- 每个目标点的最终误差
- 是否收敛
- IK 迭代次数

如果你想看全过程，再看：

- `cartesian_tracking.csv`

## 4.1 单目标实验和多目标实验的区别

- 单目标实验：只验证“能不能到一个点”
- 多目标实验：验证“多个点之间切换时，轨迹和误差是否稳定”
- 如果你刚开始调试，先用单目标
- 如果单目标已经稳定，再做多目标

### 第四步：查看结果

默认输出目录：

- `SRL2_learning/logs/cartesian_target_experiment/`

重点文件：

- `cartesian_tracking.csv`
- `cartesian_target_summary.csv`
- `cartesian_position_tracking.png`
- `cartesian_error.png`
- `cartesian_trajectory_xy.png`

## 5. 如果结果不对，先查什么

优先检查：

1. 当前 `flange frame` 参考是不是你以为的那个点
2. `tool_frame_config.json` 的单位是不是米
3. viewer 里 `tool frame` 是否真的跟着末端一起动
4. viewer 里目标点 marker 是否出现在你预期的位置
5. CSV 里的 `tool_*` 和 `target_*` 是否来自同一实验过程

## 6. 当前实现的真实性边界

严格来自 MuJoCo 的部分：

- flange 世界坐标
- tool 世界坐标
- viewer 中 world/base/flange/tool 的实时位置

教学版近似部分：

- 逆运动学是教学版数值法
- 当前没有专用工业 TCP site
- 当前默认法兰参考仍然是 `Link3 body`
- 当前主要比较位置误差，不做工业级姿态误差优化
