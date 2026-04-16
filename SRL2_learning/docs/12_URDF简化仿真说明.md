# 12_URDF简化仿真说明

## 这条链路是做什么的
这条链路专门解决一个很实际的问题：  
原来的 `Rsras1.xml` 是“人体 + 外骨骼 + 机械臂”的混合模型，坐标系复杂，而且当前仓库缺失 STL 网格，所以直接看 MuJoCo viewer 时，容易出现“人物埋进地里，只露个头”这类现象。

这次新增的链路换了一个思路：
- 不再继续强行修原始人体模型的位置
- 直接使用你提供的 `urdf_0.1.4.2.urdf` 做结构参考
- 保留关节层级、关节轴方向和关节顺序
- 在 MuJoCo 里生成一个简化的教学模型

这样做的目标不是还原原始外观，而是先把“动作、姿态、关节关系、末端轨迹”看对。

## 为什么没有直接使用原 STL 外观
原因有两个：

1. `package://` 路径不能直接在当前 MuJoCo 导入链路里稳定工作。  
2. 这个 URDF 里的 STL 面数很高，MuJoCo 直接导入时会报错，尤其是 `Link1.STL`。

所以当前版本采用的是：
- `URDF 结构` 保留
- `MuJoCo 几何外观` 简化

你可以把它理解成：
“骨架是你的，皮肤先换成教学版占位模型。”

## 为什么现在不会埋到地里
这次新链路里，基座高度是显式控制的：
- 配置文件里有 `base_height`
- 当前默认值是 `0.65`

这意味着机械臂基座会被明确放到地面上方，不再依赖原始混合模型里那一大套人体/背板/world 坐标关系。

## 当前使用的文件
- 源 URDF：
  - `e:/project/codetest/urdf_0.1.4.2/urdf/urdf_0.1.4.2.urdf`
- 新增适配器：
  - [urdf_mujoco_demo.py](/e:/project/codetest/SRL2_learning/src/srl2_learning/adapters/urdf_mujoco_demo.py)
- 新增示例入口：
  - [urdf_mujoco_demo.py](/e:/project/codetest/SRL2_learning/examples/urdf_mujoco_demo.py)
- 新增脚本入口：
  - [run_urdf_mujoco_demo.py](/e:/project/codetest/SRL2_learning/scripts/run_urdf_mujoco_demo.py)
- 新增配置：
  - [urdf_mujoco_demo_config.json](/e:/project/codetest/SRL2_learning/configs/urdf_mujoco_demo_config.json)

## 如何运行
在 `codetest` 根目录下运行：

```powershell
python SRL2_learning/scripts/run_urdf_mujoco_demo.py
```

如果要打开 MuJoCo viewer：

```powershell
python SRL2_learning/scripts/run_urdf_mujoco_demo.py --viewer
```

## 运行后会输出什么
输出目录：
- `SRL2_learning/logs/urdf_mujoco_demo/`

当前会生成：
- `urdf_replay.csv`
- `urdf_joint_tracking.png`
- `urdf_site_trajectory.png`
- `urdf_replay.gif`
- `urdf_0.1.4.2_teaching_runtime.xml`

## 当前已经确认的内容
- URDF 可以成功解析出 3 个 revolute 关节
- MuJoCo 简化模型可以成功构建
- 基座抬高后不会埋地
- 关节轨迹可以正常播放
- CSV、关节图、site 轨迹图、GIF 都能导出

## 当前明确的边界
- 这不是原始 STL 外观的完整导入版本
- 这不是原项目 `SRL2` 的完整等价仿真
- 这条链路优先解决“能稳定看动作、能分析过程、能继续教学”
- 如果后面要继续逼近真实外观，再单独处理 STL 减面、材质和更细的碰撞模型
