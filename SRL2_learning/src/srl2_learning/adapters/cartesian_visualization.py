"""四坐标系末端实验的可视化与导出模块。"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import Sequence

import matplotlib.pyplot as plt
from matplotlib import font_manager, rcParams


def _configure_matplotlib_fonts() -> None:
    available_font_names = {font.name for font in font_manager.fontManager.ttflist}
    preferred_fonts = ["Microsoft YaHei", "SimHei", "SimSun", "Microsoft JhengHei"]
    for font_name in preferred_fonts:
        if font_name in available_font_names:
            rcParams["font.sans-serif"] = [font_name]
            rcParams["axes.unicode_minus"] = False
            return
    rcParams["axes.unicode_minus"] = False


def export_cartesian_tracking_csv(output_csv_path: Path, samples: Sequence[object]) -> None:
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)
    joint_count = len(samples[0].joint_positions) if samples else 0
    headers = [
        "step_index",
        "time_seconds",
        "reference_body_name",
        "reference_site_name",
        "reference_kind",
        "tool_frame_enabled",
        "tool_label",
        "world_x",
        "world_y",
        "world_z",
        "base_x",
        "base_y",
        "base_z",
        "flange_x",
        "flange_y",
        "flange_z",
        "tool_x",
        "tool_y",
        "tool_z",
        "tool_offset_world_x",
        "tool_offset_world_y",
        "tool_offset_world_z",
        "target_x",
        "target_y",
        "target_z",
        "tool_error_x",
        "tool_error_y",
        "tool_error_z",
        "tool_error_norm",
        "experiment_case_id",
    ]
    headers.extend(f"joint_{joint_index + 1}" for joint_index in range(joint_count))

    with output_csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(headers)
        for sample in samples:
            writer.writerow(
                [
                    sample.step_index,
                    sample.time_seconds,
                    sample.reference_body_name,
                    sample.reference_site_name or "",
                    sample.reference_kind,
                    sample.tool_frame_enabled,
                    sample.tool_label,
                    *sample.world_origin,
                    *sample.base_position,
                    *sample.flange_position,
                    *sample.tool_position,
                    *sample.tool_offset_world,
                    *sample.target_position,
                    *sample.tool_error_xyz,
                    sample.tool_error_norm,
                    sample.experiment_case_id,
                    *sample.joint_positions,
                ]
            )


def export_cartesian_summary_csv(output_csv_path: Path, summaries: Sequence[object]) -> None:
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)
    headers = [
        "experiment_case_id",
        "target_index",
        "reference_body_name",
        "reference_site_name",
        "reference_kind",
        "target_x",
        "target_y",
        "target_z",
        "final_base_x",
        "final_base_y",
        "final_base_z",
        "final_flange_x",
        "final_flange_y",
        "final_flange_z",
        "final_tool_x",
        "final_tool_y",
        "final_tool_z",
        "final_tool_error_x",
        "final_tool_error_y",
        "final_tool_error_z",
        "final_tool_error_norm",
        "converged",
        "ik_iterations",
        "hold_steps",
        "tool_label",
    ]
    with output_csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(headers)
        for summary in summaries:
            writer.writerow(
                [
                    summary.experiment_case_id,
                    summary.target_index,
                    summary.reference_body_name,
                    summary.reference_site_name or "",
                    summary.reference_kind,
                    *summary.target_position,
                    *summary.final_base_position,
                    *summary.final_flange_position,
                    *summary.final_tool_position,
                    *summary.final_tool_error_xyz,
                    summary.final_tool_error_norm,
                    summary.converged,
                    summary.ik_iterations,
                    summary.hold_steps,
                    summary.tool_label,
                ]
            )


def _samples_by_case(samples: Sequence[object]) -> dict[str, list[object]]:
    grouped: dict[str, list[object]] = {}
    for sample in samples:
        grouped.setdefault(sample.experiment_case_id, []).append(sample)
    return grouped


def _axis_series(samples: Sequence[object], value_name: str, axis_index: int) -> list[float]:
    return [float(getattr(sample, value_name)[axis_index]) for sample in samples]


def _save_position_tracking_plot(output_path: Path, samples: Sequence[object], title_prefix: str) -> None:
    figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    times = [sample.time_seconds for sample in samples]
    axis_names = ["x", "y", "z"]
    for axis_index, axis in enumerate(axes):
        axis.plot(times, _axis_series(samples, "target_position", axis_index), label="目标工具点", linewidth=2.0)
        axis.plot(times, _axis_series(samples, "tool_position", axis_index), label="实际工具点", linestyle="--", linewidth=1.8)
        axis.plot(times, _axis_series(samples, "flange_position", axis_index), label="法兰点", linestyle=":", linewidth=1.4)
        axis.plot(times, _axis_series(samples, "base_position", axis_index), label="基座点", linestyle="-.", linewidth=1.2)
        axis.set_ylabel(f"{axis_names[axis_index]} (m)")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="best")
    axes[-1].set_xlabel("时间 (s)")
    figure.suptitle(f"{title_prefix} - 四坐标系位置跟踪", fontsize=14)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def _save_error_plot(output_path: Path, samples: Sequence[object], title_prefix: str) -> None:
    figure, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    times = [sample.time_seconds for sample in samples]
    axis_names = ["x", "y", "z"]
    for axis_index, axis in enumerate(axes[:3]):
        axis.plot(times, _axis_series(samples, "tool_error_xyz", axis_index), linewidth=2.0)
        axis.set_ylabel(f"e{axis_names[axis_index]} (m)")
        axis.grid(True, alpha=0.3)
    axes[3].plot(times, [sample.tool_error_norm for sample in samples], color="black", linewidth=2.0)
    axes[3].set_ylabel("||e_tool|| (m)")
    axes[3].set_xlabel("时间 (s)")
    axes[3].grid(True, alpha=0.3)
    figure.suptitle(f"{title_prefix} - 工具坐标系误差", fontsize=14)
    figure.tight_layout()
    figure.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def _save_xy_trajectory_plot(output_path: Path, samples: Sequence[object], title_prefix: str) -> None:
    figure, axis = plt.subplots(figsize=(7, 6))
    tool_x = _axis_series(samples, "tool_position", 0)
    tool_y = _axis_series(samples, "tool_position", 1)
    flange_x = _axis_series(samples, "flange_position", 0)
    flange_y = _axis_series(samples, "flange_position", 1)
    target_x = _axis_series(samples, "target_position", 0)
    target_y = _axis_series(samples, "target_position", 1)
    axis.plot(tool_x, tool_y, label="实际工具轨迹", linewidth=2.0)
    axis.plot(flange_x, flange_y, label="法兰轨迹", linestyle=":", linewidth=1.5)
    axis.scatter(tool_x[0], tool_y[0], color="green", label="起点")
    axis.scatter(tool_x[-1], tool_y[-1], color="red", label="终点")
    axis.scatter(target_x[-1], target_y[-1], color="orange", marker="x", s=80, label="目标点")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_title(f"{title_prefix} - XY 平面轨迹")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def _save_3d_trajectory_plot(output_path: Path, samples: Sequence[object], title_prefix: str) -> None:
    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(1, 1, 1, projection="3d")
    tool_x = _axis_series(samples, "tool_position", 0)
    tool_y = _axis_series(samples, "tool_position", 1)
    tool_z = _axis_series(samples, "tool_position", 2)
    flange_x = _axis_series(samples, "flange_position", 0)
    flange_y = _axis_series(samples, "flange_position", 1)
    flange_z = _axis_series(samples, "flange_position", 2)
    target_x = _axis_series(samples, "target_position", 0)
    target_y = _axis_series(samples, "target_position", 1)
    target_z = _axis_series(samples, "target_position", 2)
    axis.plot(tool_x, tool_y, tool_z, linewidth=2.0, label="实际工具轨迹")
    axis.plot(flange_x, flange_y, flange_z, linestyle=":", linewidth=1.4, label="法兰轨迹")
    axis.scatter(tool_x[0], tool_y[0], tool_z[0], color="green", label="起点")
    axis.scatter(tool_x[-1], tool_y[-1], tool_z[-1], color="red", label="终点")
    axis.scatter(target_x[-1], target_y[-1], target_z[-1], color="orange", marker="x", s=80, label="目标点")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_zlabel("z (m)")
    axis.set_title(f"{title_prefix} - 三维轨迹")
    axis.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=160, bbox_inches="tight")
    plt.close(figure)


def export_cartesian_plots(output_directory: Path, samples: Sequence[object], title_prefix: str) -> None:
    _configure_matplotlib_fonts()
    output_directory.mkdir(parents=True, exist_ok=True)
    if not samples:
        return
    _save_position_tracking_plot(output_directory / "cartesian_position_tracking.png", samples, title_prefix)
    _save_error_plot(output_directory / "cartesian_error.png", samples, title_prefix)
    _save_xy_trajectory_plot(output_directory / "cartesian_trajectory_xy.png", samples, title_prefix)
    _save_3d_trajectory_plot(output_directory / "cartesian_trajectory_3d.png", samples, title_prefix)


def export_cartesian_case_plots(output_directory: Path, samples: Sequence[object], title_prefix: str) -> None:
    for case_id, case_samples in _samples_by_case(samples).items():
        case_directory = output_directory / case_id
        export_cartesian_tracking_csv(case_directory / "cartesian_tracking.csv", case_samples)
        export_cartesian_plots(case_directory, case_samples, title_prefix=f"{title_prefix} - {case_id}")
