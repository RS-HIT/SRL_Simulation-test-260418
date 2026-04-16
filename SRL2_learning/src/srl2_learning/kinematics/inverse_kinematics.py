"""教学版数值逆运动学。

当前方法：
- Damped Least Squares
- 位置误差驱动
- Jacobian 用有限差分近似

这不是工业级 IK 求解器，但足够用于教学版“目标点 -> 关节角”实验。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .forward_kinematics import compute_forward_kinematics
from .model_context import MujocoKinematicModelContext
from .tool_frame import ToolFrameConfig


@dataclass(slots=True)
class IKSolveResult:
    """逆运动学求解结果。"""

    joint_positions: list[float]
    final_position: list[float]
    position_error_xyz: list[float]
    position_error_norm: float
    iterations: int
    converged: bool


class DampedLeastSquaresIKSolver:
    """阻尼最小二乘逆运动学求解器。"""

    def __init__(
        self,
        model_context: MujocoKinematicModelContext,
        finite_difference_step: float = 1e-4,
    ) -> None:
        self.model_context = model_context
        self.finite_difference_step = float(finite_difference_step)

    def _compute_position_jacobian(
        self,
        joint_positions: np.ndarray,
        tool_frame: ToolFrameConfig | None,
    ) -> np.ndarray:
        """用有限差分计算位置雅可比。"""
        base_result = compute_forward_kinematics(
            context=self.model_context,
            joint_positions=joint_positions.tolist(),
            tool_frame=tool_frame,
        )
        base_position = np.array(base_result.tcp_position, dtype=float)
        jacobian = np.zeros((3, len(joint_positions)), dtype=float)

        for joint_index in range(len(joint_positions)):
            perturbed = joint_positions.copy()
            perturbed[joint_index] += self.finite_difference_step
            perturbed = self.model_context.clamp_joint_positions(perturbed)
            perturbed_result = compute_forward_kinematics(
                context=self.model_context,
                joint_positions=perturbed.tolist(),
                tool_frame=tool_frame,
            )
            perturbed_position = np.array(perturbed_result.tcp_position, dtype=float)
            jacobian[:, joint_index] = (perturbed_position - base_position) / self.finite_difference_step

        return jacobian

    def solve_inverse_kinematics(
        self,
        target_xyz: list[float],
        initial_joint_positions: list[float],
        tool_frame: ToolFrameConfig | None = None,
        max_iterations: int = 200,
        tolerance: float = 1e-3,
        step_size: float = 0.8,
        damping: float = 1e-2,
    ) -> IKSolveResult:
        """求解目标点对应的关节角。"""
        target = np.array(target_xyz, dtype=float)
        current = self.model_context.clamp_joint_positions(np.array(initial_joint_positions, dtype=float))

        best_joint_positions = current.copy()
        best_position = target.copy()
        best_error_vector = np.array([float("inf"), float("inf"), float("inf")], dtype=float)
        best_error_norm = float("inf")
        converged = False

        for iteration_index in range(max_iterations):
            fk_result = compute_forward_kinematics(
                context=self.model_context,
                joint_positions=current.tolist(),
                tool_frame=tool_frame,
            )
            current_position = np.array(fk_result.tcp_position, dtype=float)
            error_vector = target - current_position
            error_norm = float(np.linalg.norm(error_vector))

            if error_norm < best_error_norm:
                best_joint_positions = current.copy()
                best_position = current_position.copy()
                best_error_vector = error_vector.copy()
                best_error_norm = error_norm

            if error_norm <= tolerance:
                converged = True
                return IKSolveResult(
                    joint_positions=current.tolist(),
                    final_position=current_position.tolist(),
                    position_error_xyz=error_vector.tolist(),
                    position_error_norm=error_norm,
                    iterations=iteration_index + 1,
                    converged=True,
                )

            jacobian = self._compute_position_jacobian(current, tool_frame=tool_frame)
            jtj = jacobian.T @ jacobian
            damping_matrix = (float(damping) ** 2) * np.eye(jtj.shape[0], dtype=float)
            delta_joint = float(step_size) * np.linalg.solve(jtj + damping_matrix, jacobian.T @ error_vector)
            current = self.model_context.clamp_joint_positions(current + delta_joint)

        return IKSolveResult(
            joint_positions=best_joint_positions.tolist(),
            final_position=best_position.tolist(),
            position_error_xyz=best_error_vector.tolist(),
            position_error_norm=best_error_norm,
            iterations=max_iterations,
            converged=converged,
        )


def solve_inverse_kinematics(
    model_context: MujocoKinematicModelContext,
    target_xyz: list[float],
    initial_joint_positions: list[float],
    tool_frame: ToolFrameConfig | None = None,
    max_iterations: int = 200,
    tolerance: float = 1e-3,
    step_size: float = 0.8,
    damping: float = 1e-2,
) -> IKSolveResult:
    """便捷逆运动学接口。"""
    solver = DampedLeastSquaresIKSolver(model_context=model_context)
    return solver.solve_inverse_kinematics(
        target_xyz=target_xyz,
        initial_joint_positions=initial_joint_positions,
        tool_frame=tool_frame,
        max_iterations=max_iterations,
        tolerance=tolerance,
        step_size=step_size,
        damping=damping,
    )
