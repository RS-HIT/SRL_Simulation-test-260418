# MuJoCo 复现说明

## 这份说明解决什么问题
学习版现在已经可以利用原项目 `SRL2` 中留下的 MuJoCo 模型结构做动作回放，但它不是“直接把原工程搬过来”。这份文档专门说明：
- 当前到底复现了什么。
- 为什么需要运行时清洗 XML。
- 学习版和原项目在 MuJoCo 这部分是怎么一一对应的。

## 当前复现到了哪一步
第一版已经实现：
- 读取原始模型 `SRL2/model/Rsras1.xml`
- 运行时清洗绝对 `meshdir`
- 删除缺失 STL 依赖的 `<mesh>`、mesh geom 和相关 `<contact>`
- 保留原 XML 中的关节层级、人体层级、site 和 sensor
- 按原 `main.cpp` 的 `qpos` 写法驱动右侧 SRAs 六个关节
- 使用 MuJoCo viewer 做实时回放
- 使用 MuJoCo Renderer 做离线 GIF、CSV 和图表导出

## 为什么当前是“无网格结构回放”
根因不是 MuJoCo 不能跑，而是原模型依赖的 STL 网格没有放进当前仓库。

已确认事实：
- `SRL2/model/Rsras1.xml` 写死了 Linux 绝对 `meshdir`
- XML 中引用了 `back.STL`、`Link1_r.STL` 等网格
- 当前仓库内没有这些 STL 文件

所以第一版采用的策略是：
- 不改 `SRL2`
- 不伪造新模型
- 运行时把 mesh 和依赖 mesh 的 contact 去掉
- 保留关节、body、site、sensor 和 `qpos` 布局

这样做的结果是：
- 外观会简化
- 结构仍然对得上原项目
- 动作回放和末端轨迹观察仍然成立

## 和原项目的对应关系
### 1. 模型入口
- 原项目：`SRL2/main.cpp` 中调用 `m_mujoco.init("./Rsras1.xml")`
- 学习版：`SRL2MujocoModelAdapter` 读取 `SRL2/model/Rsras1.xml`

### 2. 姿态写入方式
原项目在 `main.cpp` 中按固定槽位写 `qpos`：
- `qpos[0:6]`：背板自由关节
- `qpos[7:12]`：右 SRAs 六关节
- `qpos[13:18]`：左 SRAs 六关节
- `qpos[19:23]`：右人体肩/肘
- `qpos[24:28]`：左人体肩/肘

学习版保持同样布局，对应模块是：
- `SRL2QposWriter`

### 3. MuJoCo 更新方式
- 原项目：`mujocosim.cpp` 里主要调用 `mj_forward`
- 学习版：每一帧写入 `qpos` 后也调用 `mujoco.mj_forward`

## 当前动作来源
第一版动作不是直接复现 `gohome`、`supporting` 这类原项目场景函数，而是：
- 先复用学习版现有最小主链路
- 让 `LearningRobotController.run_joint_space_motion(...)` 生成一段右臂关节轨迹
- 再把这段轨迹写进 MuJoCo

这样做的原因很直接：
- 先保证一条稳定、可验证、可观察的回放链路跑通
- 避免第一版就把原项目状态机、硬件依赖和场景函数一起拖进来

## 当前输出文件
运行：

```powershell
python SRL2_learning/scripts/run_mujoco_demo.py
```

会在 `SRL2_learning/logs/mujoco_demo/` 下生成：
- `mujoco_replay.csv`
  - 每一步的时间、右臂目标角、完整 `qpos`、关键 site 坐标
- `right_arm_joint_tracking.png`
  - 右臂六关节目标角和 MuJoCo `qpos` 对照图
- `site_trajectory.png`
  - `robot_tip_r`、`arm_tip_r` 的轨迹图
- `mujoco_replay.gif`
  - 离线渲染出来的动作回放
- `Rsras1_meshless_runtime.xml`
  - 运行时清洗后的 XML，仅用于调试，不回写到 `SRL2`

## 当前明确没有做的事
第一版暂不包含：
- 原项目完整状态机
- T265 人体姿态驱动
- 接触逻辑复现
- 人体肩肘联动回放
- 原始 STL 外观恢复
- `gohome` / `supporting` / `action1` 等场景动作复现

## 第二阶段可以怎么扩
后续如果继续扩展，建议顺序是：
1. 从原 `main.cpp` 里挑 1 到 3 个固定动作函数做离线复现
2. 再把人体肩关节四元数和肘角接进 `qpos[19:28]`
3. 最后再考虑 contact、sensor 和更接近原流程的状态切换
