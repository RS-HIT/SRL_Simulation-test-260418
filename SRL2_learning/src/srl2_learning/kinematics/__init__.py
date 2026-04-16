"""教学版运动学子包。"""
"""运动学子包。

这个子包在整条教学链路里的位置：
- 它位于“末端目标输入”和“关节轨迹规划”之间。
- 它负责把“我想让末端去哪里”翻译成“机械臂各个关节应该转到多少”。

为什么需要它：
- 现有主链路更擅长处理“关节空间目标”，也就是已经知道每个关节该转多少。
- 但很多实验更自然的输入其实是“末端想去某个三维坐标”。
- 所以这里补了一层运动学，把末端目标和关节目标连接起来。

当前边界：
- v1 先做位置运动学，不把姿态误差纳入优化目标。
- 正运动学位置读取严格依赖 MuJoCo 模型当前位姿。
- 逆运动学使用教学版数值法，目标是可读、稳定、便于实验，不是工业级求解器。
"""

from .forward_kinematics import ForwardKinematicsResult, compute_forward_kinematics
from .inverse_kinematics import DampedLeastSquaresIKSolver, IKSolveResult, solve_inverse_kinematics
from .model_context import MujocoKinematicModelContext, load_real_mesh_model_context
from .tool_frame import ToolFrameConfig, load_tool_frame_config

__all__ = [
    "DampedLeastSquaresIKSolver",
    "ForwardKinematicsResult",
    "IKSolveResult",
    "MujocoKinematicModelContext",
    "ToolFrameConfig",
    "compute_forward_kinematics",
    "load_real_mesh_model_context",
    "load_tool_frame_config",
    "solve_inverse_kinematics",
]
