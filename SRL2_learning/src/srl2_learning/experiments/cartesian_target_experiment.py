"""四坐标系末端实验执行模块。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

from ..adapters.urdf_mujoco_demo import build_joint_trajectory
from ..kinematics.forward_kinematics import ForwardKinematicsResult, compute_forward_kinematics
from ..kinematics.inverse_kinematics import IKSolveResult, solve_inverse_kinematics
from ..kinematics.model_context import MujocoKinematicModelContext
from ..kinematics.tool_frame import ToolFrameConfig


@dataclass(slots=True)
class CartesianTrackingSample:
    step_index: int
    time_seconds: float
    world_origin: list[float]
    base_position: list[float]
    flange_position: list[float]
    tool_position: list[float]
    target_position: list[float]
    actual_position: list[float]
    tool_error_xyz: list[float]
    tool_error_norm: float
    tool_offset_world: list[float]
    reference_body_name: str
    reference_site_name: str | None
    reference_kind: str
    tool_frame_enabled: bool
    tool_label: str
    joint_positions: list[float]
    experiment_case_id: str
    tcp_position: list[float]
    tcp_error_xyz: list[float]
    tcp_error_norm: float


@dataclass(slots=True)
class CartesianTargetSummary:
    experiment_case_id: str
    target_index: int
    target_position: list[float]
    solved_joint_positions: list[float]
    final_base_position: list[float]
    final_flange_position: list[float]
    final_tool_position: list[float]
    final_position: list[float]
    final_tool_error_xyz: list[float]
    final_tool_error_norm: float
    reference_body_name: str
    reference_site_name: str | None
    reference_kind: str
    converged: bool
    ik_iterations: int
    hold_steps: int
    tool_label: str
    final_tcp_position: list[float]
    final_tcp_error_xyz: list[float]
    final_tcp_error_norm: float


@dataclass(slots=True)
class CartesianExperimentResult:
    experiment_mode: str
    samples: list[CartesianTrackingSample]
    summaries: list[CartesianTargetSummary]
    ik_results: list[IKSolveResult]
    joint_trajectory: list[list[float]]
    target_positions: list[list[float]]


def _as_float_list(values: Iterable[float]) -> list[float]:
    return [float(value) for value in values]


def _resolve_tool_frame(tool_frame: ToolFrameConfig | None, use_tool_frame: bool) -> ToolFrameConfig | None:
    if tool_frame is None or not use_tool_frame or not tool_frame.enabled:
        return None
    return tool_frame


def _sample_from_fk_result(
    step_index: int,
    control_period: float,
    target_position: Sequence[float],
    joint_positions: Sequence[float],
    fk_result: ForwardKinematicsResult,
    experiment_case_id: str,
    tool_label: str,
    tool_frame_enabled: bool,
) -> CartesianTrackingSample:
    target_array = np.asarray(target_position, dtype=float)
    tool_array = np.asarray(fk_result.tool_position, dtype=float)
    tool_error = target_array - tool_array
    return CartesianTrackingSample(
        step_index=step_index,
        time_seconds=step_index * float(control_period),
        world_origin=fk_result.world_origin.copy(),
        base_position=fk_result.base_position.copy(),
        flange_position=fk_result.flange_position.copy(),
        tool_position=fk_result.tool_position.copy(),
        target_position=_as_float_list(target_array),
        actual_position=fk_result.tool_position.copy(),
        tool_error_xyz=_as_float_list(tool_error),
        tool_error_norm=float(np.linalg.norm(tool_error)),
        tool_offset_world=fk_result.tool_offset_world.copy(),
        reference_body_name=fk_result.reference_body_name,
        reference_site_name=fk_result.reference_site_name,
        reference_kind=fk_result.reference_kind,
        tool_frame_enabled=tool_frame_enabled,
        tool_label=tool_label,
        joint_positions=_as_float_list(joint_positions),
        experiment_case_id=experiment_case_id,
        tcp_position=fk_result.tool_position.copy(),
        tcp_error_xyz=_as_float_list(tool_error),
        tcp_error_norm=float(np.linalg.norm(tool_error)),
    )


def _build_tracking_samples(
    model_context: MujocoKinematicModelContext,
    joint_trajectory: Sequence[Sequence[float]],
    target_position: Sequence[float],
    control_period: float,
    experiment_case_id: str,
    tool_label: str,
    tool_frame: ToolFrameConfig | None,
    start_step_index: int,
) -> list[CartesianTrackingSample]:
    samples: list[CartesianTrackingSample] = []
    for local_step_index, joint_positions in enumerate(joint_trajectory):
        fk_result = compute_forward_kinematics(model_context, list(joint_positions), tool_frame)
        samples.append(
            _sample_from_fk_result(
                step_index=start_step_index + local_step_index,
                control_period=control_period,
                target_position=target_position,
                joint_positions=joint_positions,
                fk_result=fk_result,
                experiment_case_id=experiment_case_id,
                tool_label=tool_label,
                tool_frame_enabled=tool_frame is not None,
            )
        )
    return samples


def _build_joint_motion_segment(
    start_joint_positions: Sequence[float],
    target_joint_positions: Sequence[float],
    control_period: float,
    max_velocity: float,
    max_acceleration: float,
    hold_steps: int,
) -> list[list[float]]:
    segment = build_joint_trajectory(
        start_joint_position=start_joint_positions,
        target_joint_position=target_joint_positions,
        control_period=control_period,
        max_velocity=max_velocity,
        max_acceleration=max_acceleration,
    )
    if hold_steps > 0 and segment:
        segment.extend([segment[-1].copy() for _ in range(int(hold_steps))])
    return [[float(value) for value in joint_positions] for joint_positions in segment]


def _build_summary(
    experiment_case_id: str,
    target_index: int,
    target_position: Sequence[float],
    ik_result: IKSolveResult,
    final_sample: CartesianTrackingSample,
    hold_steps: int,
) -> CartesianTargetSummary:
    return CartesianTargetSummary(
        experiment_case_id=experiment_case_id,
        target_index=target_index,
        target_position=_as_float_list(target_position),
        solved_joint_positions=_as_float_list(ik_result.joint_positions),
        final_base_position=final_sample.base_position.copy(),
        final_flange_position=final_sample.flange_position.copy(),
        final_tool_position=final_sample.tool_position.copy(),
        final_position=final_sample.tool_position.copy(),
        final_tool_error_xyz=final_sample.tool_error_xyz.copy(),
        final_tool_error_norm=final_sample.tool_error_norm,
        reference_body_name=final_sample.reference_body_name,
        reference_site_name=final_sample.reference_site_name,
        reference_kind=final_sample.reference_kind,
        converged=ik_result.converged,
        ik_iterations=ik_result.iterations,
        hold_steps=int(hold_steps),
        tool_label=final_sample.tool_label,
        final_tcp_position=final_sample.tool_position.copy(),
        final_tcp_error_xyz=final_sample.tool_error_xyz.copy(),
        final_tcp_error_norm=final_sample.tool_error_norm,
    )


def run_single_target_experiment(
    model_context: MujocoKinematicModelContext,
    target_position: Sequence[float],
    initial_joint_positions: Sequence[float],
    control_period: float,
    max_velocity: float,
    max_acceleration: float,
    ik_max_iterations: int,
    ik_tolerance: float,
    ik_step_size: float,
    ik_damping: float,
    hold_steps: int = 0,
    tool_frame: ToolFrameConfig | None = None,
    use_tool_frame: bool = False,
    experiment_case_id: str = "target_1",
) -> CartesianExperimentResult:
    active_tool_frame = _resolve_tool_frame(tool_frame, use_tool_frame)
    clamped_initial_joint_positions = _as_float_list(
        model_context.clamp_joint_positions(np.asarray(initial_joint_positions, dtype=float))
    )
    ik_result = solve_inverse_kinematics(
        model_context=model_context,
        target_xyz=target_position,
        initial_joint_positions=clamped_initial_joint_positions,
        tool_frame=active_tool_frame,
        max_iterations=ik_max_iterations,
        tolerance=ik_tolerance,
        step_size=ik_step_size,
        damping=ik_damping,
    )
    joint_trajectory = _build_joint_motion_segment(
        start_joint_positions=clamped_initial_joint_positions,
        target_joint_positions=ik_result.joint_positions,
        control_period=control_period,
        max_velocity=max_velocity,
        max_acceleration=max_acceleration,
        hold_steps=hold_steps,
    )
    samples = _build_tracking_samples(
        model_context=model_context,
        joint_trajectory=joint_trajectory,
        target_position=target_position,
        control_period=control_period,
        experiment_case_id=experiment_case_id,
        tool_label=active_tool_frame.label if active_tool_frame else "no_tool_offset",
        tool_frame=active_tool_frame,
        start_step_index=0,
    )
    return CartesianExperimentResult(
        experiment_mode="single",
        samples=samples,
        summaries=[_build_summary(experiment_case_id, 0, target_position, ik_result, samples[-1], hold_steps)],
        ik_results=[ik_result],
        joint_trajectory=joint_trajectory,
        target_positions=[_as_float_list(target_position)],
    )


def run_multi_target_experiment(
    model_context: MujocoKinematicModelContext,
    target_points: Sequence[Sequence[float]],
    initial_joint_positions: Sequence[float],
    control_period: float,
    max_velocity: float,
    max_acceleration: float,
    ik_max_iterations: int,
    ik_tolerance: float,
    ik_step_size: float,
    ik_damping: float,
    hold_steps: int = 0,
    tool_frame: ToolFrameConfig | None = None,
    use_tool_frame: bool = False,
) -> CartesianExperimentResult:
    active_tool_frame = _resolve_tool_frame(tool_frame, use_tool_frame)
    current_joint_positions = _as_float_list(
        model_context.clamp_joint_positions(np.asarray(initial_joint_positions, dtype=float))
    )
    all_samples: list[CartesianTrackingSample] = []
    all_summaries: list[CartesianTargetSummary] = []
    all_ik_results: list[IKSolveResult] = []
    all_joint_trajectory: list[list[float]] = []
    step_offset = 0

    for target_index, target_position in enumerate(target_points):
        experiment_case_id = f"target_{target_index + 1}"
        ik_result = solve_inverse_kinematics(
            model_context=model_context,
            target_xyz=target_position,
            initial_joint_positions=current_joint_positions,
            tool_frame=active_tool_frame,
            max_iterations=ik_max_iterations,
            tolerance=ik_tolerance,
            step_size=ik_step_size,
            damping=ik_damping,
        )
        segment = _build_joint_motion_segment(
            start_joint_positions=current_joint_positions,
            target_joint_positions=ik_result.joint_positions,
            control_period=control_period,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            hold_steps=hold_steps,
        )
        segment_samples = _build_tracking_samples(
            model_context=model_context,
            joint_trajectory=segment,
            target_position=target_position,
            control_period=control_period,
            experiment_case_id=experiment_case_id,
            tool_label=active_tool_frame.label if active_tool_frame else "no_tool_offset",
            tool_frame=active_tool_frame,
            start_step_index=step_offset,
        )
        all_samples.extend(segment_samples)
        all_joint_trajectory.extend(segment)
        all_ik_results.append(ik_result)
        all_summaries.append(_build_summary(experiment_case_id, target_index, target_position, ik_result, segment_samples[-1], hold_steps))
        current_joint_positions = _as_float_list(ik_result.joint_positions)
        step_offset += len(segment)

    return CartesianExperimentResult(
        experiment_mode="multi",
        samples=all_samples,
        summaries=all_summaries,
        ik_results=all_ik_results,
        joint_trajectory=all_joint_trajectory,
        target_positions=[_as_float_list(point) for point in target_points],
    )


def run_tcp_sensitivity_experiment(
    model_context: MujocoKinematicModelContext,
    target_position: Sequence[float],
    initial_joint_positions: Sequence[float],
    tcp_cases: Sequence[ToolFrameConfig],
    control_period: float,
    max_velocity: float,
    max_acceleration: float,
    ik_max_iterations: int,
    ik_tolerance: float,
    ik_step_size: float,
    ik_damping: float,
    hold_steps: int = 0,
) -> list[CartesianExperimentResult]:
    results: list[CartesianExperimentResult] = []
    for case_index, tcp_case in enumerate(tcp_cases):
        result = run_single_target_experiment(
            model_context=model_context,
            target_position=target_position,
            initial_joint_positions=initial_joint_positions,
            control_period=control_period,
            max_velocity=max_velocity,
            max_acceleration=max_acceleration,
            ik_max_iterations=ik_max_iterations,
            ik_tolerance=ik_tolerance,
            ik_step_size=ik_step_size,
            ik_damping=ik_damping,
            hold_steps=hold_steps,
            tool_frame=tcp_case,
            use_tool_frame=True,
            experiment_case_id=f"tool_case_{case_index + 1}",
        )
        result.experiment_mode = "tool_sensitivity"
        results.append(result)
    return results
