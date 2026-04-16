"""学习版常量表。

这个文件有什么用：
- 把零散的基础数字集中到一个地方，避免在代码里到处出现“看不懂的神秘数字”。
- 新手读代码时，只要先知道这些常量代表什么，后面的公式就容易很多。

这个文件在主流程里的角色：
- 它不是直接执行某一步的模块，更像“公共说明牌”。
- 映射层、控制层、规划层都会读取这里的常量。

为什么要单独放一个文件：
- 如果把这些数字散在各个函数里，新手很难判断“这是临时写死的数字”还是“有物理意义的固定参数”。
- 集中管理后，也更容易对照原项目来源。
"""

from __future__ import annotations

# 圆周率。
# 类型：float
# 例子：PI 约等于 3.1415926。
# 用途：角度和弧度互相换算时会用到。
PI = 3.1415926

# 控制模式编号。
# 类型：int
# 例子：POSITION_MODE = 1 代表位置模式。
# 说明：学习版当前没有完整用到这些模式，但保留它们有助于和原项目术语对齐。
POSITION_MODE = 1
VELOCITY_MODE = 2
TORQUE_MODE = 3

# 前三个关键关节的减速比。
# 类型：float
# 例子：如果关节 1 转过 1 弧度，电机侧需要转 JOINT_1_REDUCTION_RATIO 倍的角度。
# 说明：这些比值直接参与 joint <-> motor 映射公式。
JOINT_1_REDUCTION_RATIO = 4.15392189
JOINT_2_REDUCTION_RATIO = 4.15392189
JOINT_3_REDUCTION_RATIO = 2.043649491

# 关节位置上限。
# 类型：list[float]
# 单位：弧度
# 例子：30 * PI / 180 表示 30 度换算后的弧度。
# 说明：当前学习版最小链路没有强制裁剪这些限位，但保留这些常量有助于后续教学扩展。
JOINT_POSITION_UPPER_LIMITS = [
    30 * PI / 180,
    120 * PI / 180,
    30 * PI / 180,
    90 * PI / 180,
    90 * PI / 180,
    90 * PI / 180,
]

# 关节位置下限。
# 类型：list[float]
# 单位：弧度
JOINT_POSITION_LOWER_LIMITS = [
    -180 * PI / 180,
    -85 * PI / 180,
    -175 * PI / 180,
    -90 * PI / 180,
    -90 * PI / 180,
    -90 * PI / 180,
]

# 执行层保护阈值。
# 类型：float
# 说明：这些值来自原项目的保护参数。学习版当前主要用于文档说明和后续扩展参考。
MOTOR_TORQUE_LIMIT = 9
MOTOR_VELOCITY_LIMIT = 800 * PI / 180
MOTOR_POSITION_LIMIT = 4 * PI / 180

# 学习版默认配置。
# 类型：float / int
# DEFAULT_CONTROL_PERIOD：控制周期，单位秒。例子：0.005 表示 5 毫秒一拍。
# DEFAULT_JOINT_DOF：主关节自由度数量。
# DEFAULT_COMMAND_DOF：完整命令向量长度。之所以比关节自由度多 1，是为了保留和原项目更接近的命令形状。
DEFAULT_CONTROL_PERIOD = 0.005
DEFAULT_JOINT_DOF = 6
DEFAULT_COMMAND_DOF = 7
