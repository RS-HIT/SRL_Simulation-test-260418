# AGENT.md

这份文件用于向 ChatGPT、Codex、Claude 或其他代码助手快速说明 `SRL2_learning` 当前的工程状态，避免重复摸索背景。

## 项目身份
- 项目名：`SRL2_learning`
- 工作空间根目录：`codetest`
- 原始参考目录：`codetest/SRL2`
- 重要边界：`SRL2` 是只读参考源码，不允许修改
- 所有新增和修改都只能发生在 `codetest/SRL2_learning`

## 项目目标
`SRL2_learning` 不是在“优化原项目”，而是在制作原项目的教学友好镜像版本。

核心目标：
- 保留原项目核心功能和思路
- 降低新手理解门槛
- 用更清晰的命名、中文注释和实验脚本帮助理解
- 所有不确定和未验证部分必须明确标注，不允许假装等价原版

## 当前已经实现的内容

### 1. 文档体系
已存在的核心文档：
- `docs/01_项目结构分析.md`
- `docs/02_原项目模块映射表.md`
- `docs/03_功能对照表.md`
- `docs/04_命名重构对照表.md`
- `docs/05_阅读路线.md`
- `docs/06_迁移验证报告.md`
- `docs/07_超级新手入门总览.md`
- `docs/08_主流程图解.md`
- `docs/09_关键函数说明书.md`
- `docs/10_三天入门计划.md`
- `docs/11_MuJoCo复现说明.md`
- `docs/12_URDF简化仿真说明.md`
- `docs/13_URDF真实网格仿真说明.md`
- `docs/14_末端坐标驱动实验说明.md`

### 2. 最小关节空间链路
核心模块位于：
- `src/srl2_learning/core/`

已实现：
- 轨迹规划
- 状态更新
- 动力学接口壳
- 关节到电机映射
- 执行记录
- 离线 CSV 和图像导出

### 3. MuJoCo 相关链路
已实现三条相关链路：

#### A. 原模型结构回放
- 参考 `SRL2/model/Rsras1.xml`
- 参考原 `main.cpp` 的 `qpos` 布局
- 当前主要用于结构级回放和教学观察

#### B. URDF 简化仿真
- 早期教学链路
- 用简化几何体保留关节层级和动作

#### C. URDF 真实网格仿真
- 当前优先链路
- 读取外部 `urdf_0.1.4.2.urdf`
- 自动镜像外部模型到 `SRL2_learning/assets/`
- 对超出 MuJoCo 限制的 STL 自动减面
- 生成运行时 URDF / MJCF
- 支持 viewer、CSV、图片、GIF

### 4. 末端坐标驱动实验
已实现位置型末端实验：
- 单目标点到达
- 多目标点顺序切换
- TCP 偏移敏感性实验

对应模块：
- `src/srl2_learning/kinematics/`
- `src/srl2_learning/experiments/cartesian_target_experiment.py`
- `src/srl2_learning/adapters/cartesian_visualization.py`
- `scripts/run_cartesian_target_experiment.py`

## 当前关键事实

### 关于末端实验
- 默认目标坐标系：世界坐标系
- 默认末端参考：`body=Link3`
- 原因：当前模型没有稳定专用末端 `site`
- TCP 支持：平移偏移已接入；旋转字段保留，但 v1 位置实验主要使用平移
- 正运动学：基于 MuJoCo 模型位姿读取
- 逆运动学：教学版 `damped least squares`

### 关于 Viewer 和场景
- 当前 viewer 应优先加载运行时 scene / MJCF，而不是直接裸读 URDF
- 背景、地板、灯光等视觉元素是后来补进最外层 scene 的
- 如果出现背景异常、相机错误、模型埋地等问题，优先检查配置和运行时 scene 文件

### 关于配置
重点配置文件：
- `configs/urdf_real_mesh_demo_config.json`
- `configs/cartesian_target_experiment.json`
- `configs/tool_frame_config.json`
- `configs/tcp_sensitivity_cases.json`

## 已验证状态
当前已验证：
- 最小关节空间示例可运行
- MuJoCo 原模型回放链路可运行
- URDF 真实网格链路可运行
- 末端坐标实验三种模式可运行
- `python SRL2_learning/scripts/run_validation.py` 当前通过

## 明确不是原版等价实现的部分
- `LinearInterpolationTrajectoryPlanner` 不是原版 Reflexxes
- `OfflineDynamicsService` 不是原版完整 RBDL
- 末端坐标实验 IK 不是工业级求解器
- 当前“实际末端位置”不是实机闭环反馈
- 当前 3 关节 URDF 不是原始完整人体/外骨骼系统

## 已知问题和常见误区

### 1. README 以前出现过中文乱码
- 原因大概率是编码和终端显示混用
- 现在建议统一按 UTF-8 维护

### 2. 真实网格模型依赖外部 URDF 和 STL
- 当前链路依赖 `urdf_0.1.4.2` 目录存在
- 处理后的网格和运行时模型会写入 `SRL2_learning/assets/external_models/urdf_0.1.4.2/`

### 3. 改姿态参数可能触发模型重建
- 优先通过配置修改
- 若缓存策略失效，可能重新走预处理链

### 4. 不要把“关节目标”和“末端目标”混为一谈
- 关节空间实验：输入是每个关节的目标角
- 末端实验：输入是末端希望到达的 `(x, y, z)`

## 如果你要向 ChatGPT 反馈问题，建议这样描述
请尽量带上下面这些信息：

### 基本上下文
- 当前目录：`codetest/SRL2_learning`
- 不能修改：`codetest/SRL2`
- 当前运行的是哪个脚本
- 使用的是哪个配置文件

### 最小复现命令
例如：
```powershell
python SRL2_learning/scripts/run_cartesian_target_experiment.py --experiment-mode tcp_sensitivity
```

### 实际现象
例如：
- viewer 没弹出
- 机械臂朝向不对
- 背景纯黑
- IK 不收敛
- 导出的图为空

### 你希望的结果
例如：
- 希望 viewer 默认打开
- 希望用官方风格背景和地板
- 希望末端实验支持姿态约束
- 希望默认目标点换成更容易收敛的位置

### 最好附带的信息
- 报错堆栈
- 配置文件内容
- 运行后生成的文件路径
- 如果是 MuJoCo 画面问题，最好附截图

## 建议 ChatGPT 优先检查的文件
如果问题和末端实验有关，优先看：
- `scripts/run_cartesian_target_experiment.py`
- `src/srl2_learning/kinematics/model_context.py`
- `src/srl2_learning/kinematics/forward_kinematics.py`
- `src/srl2_learning/kinematics/inverse_kinematics.py`
- `src/srl2_learning/experiments/cartesian_target_experiment.py`
- `src/srl2_learning/adapters/cartesian_visualization.py`
- `configs/cartesian_target_experiment.json`
- `configs/tool_frame_config.json`

如果问题和 URDF / MuJoCo viewer 有关，优先看：
- `scripts/run_urdf_real_mesh_demo.py`
- `src/srl2_learning/adapters/urdf_real_mesh_mujoco.py`
- `src/srl2_learning/adapters/urdf_mujoco_demo.py`
- `configs/urdf_real_mesh_demo_config.json`

## 当前建议
- 如果目标是继续教学化，优先补文档、注释和实验结果解释
- 如果目标是继续逼近真实工程，再考虑姿态 IK、专用末端 site、碰撞约束和更完整的动力学验证
