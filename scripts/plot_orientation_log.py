#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CSV_PATH = ROOT / "logs" / "refactored_teleop_20260728_162634.csv"


def load_log(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV is empty: {path}")
    return rows


def values(rows, name, default=0.0):
    return np.array([float(row.get(name, default)) for row in rows])


def values_any(rows, *names):
    for name in names:
        if name in rows[0]:
            return values(rows, name)
    return np.zeros(len(rows))


def rotvec_norm_deg(rows, prefix):
    components = np.column_stack([
        values(rows, f"{prefix}_x"),
        values(rows, f"{prefix}_y"),
        values(rows, f"{prefix}_z"),
    ])
    return np.rad2deg(np.linalg.norm(components, axis=1))


def resolve_csv_path():
    if len(sys.argv) > 1:
        return Path(sys.argv[1]).expanduser().resolve()
    candidates = sorted(ROOT.joinpath("logs").glob("refactored_teleop_*.csv"))
    if candidates:
        return candidates[-1]
    return DEFAULT_CSV_PATH


def shade_inactive_intervals(axis, time_s, teleop_active):
    inactive = ~teleop_active
    if np.any(inactive):
        axis.fill_between(
            time_s,
            0,
            1,
            where=inactive,
            transform=axis.get_xaxis_transform(),
            color="0.7",
            alpha=0.25,
            label="teleop inactive",
        )


def plot_signed_position_diagnostics(rows, time_s, csv_path):
    teleop_active = values(rows, "teleop_active") > 0.5
    initial_command = np.array([
        values(rows, "fr3_command_position_x")[0],
        values(rows, "fr3_command_position_y")[0],
        values(rows, "fr3_command_position_z")[0],
    ])
    initial_target = np.array([
        values(rows, "fr3_target_position_x")[0],
        values(rows, "fr3_target_position_y")[0],
        values(rows, "fr3_target_position_z")[0],
    ])

    series = {
        "OMY clutch-relative position": np.column_stack([
            values(rows, "omy_delta_position_x_m"),
            values(rows, "omy_delta_position_y_m"),
            values(rows, "omy_delta_position_z_m"),
        ]),
        "Mapped FR3 position increment": np.column_stack([
            values(rows, "mapped_position_delta_x_m"),
            values(rows, "mapped_position_delta_y_m"),
            values(rows, "mapped_position_delta_z_m"),
        ]),
        "FR3 command position from session initial": np.column_stack([
            values(rows, "fr3_command_position_x") - initial_command[0],
            values(rows, "fr3_command_position_y") - initial_command[1],
            values(rows, "fr3_command_position_z") - initial_command[2],
        ]),
        "FR3 target position from session initial": np.column_stack([
            values(rows, "fr3_target_position_x") - initial_target[0],
            values(rows, "fr3_target_position_y") - initial_target[1],
            values(rows, "fr3_target_position_z") - initial_target[2],
        ]),
    }

    figure, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    figure.suptitle(
        f"Signed Position Axis Diagnostics\n{csv_path.name}"
    )
    labels = ("X", "Y", "Z")
    colors = ("tab:red", "tab:green", "tab:blue")

    for axis, (title, data) in zip(axes.flat, series.items()):
        for index, label in enumerate(labels):
            axis.plot(
                time_s,
                data[:, index] * 1000.0,
                color=colors[index],
                label=label,
            )
        shade_inactive_intervals(axis, time_s, teleop_active)
        axis.set_title(title)
        axis.set_ylabel("Delta [mm]")
        axis.grid(True)
        axis.legend()

    for axis in axes[1, :]:
        axis.set_xlabel("Time [s]")

    figure.tight_layout()
    output_path = csv_path.with_name(f"{csv_path.stem}_signed_position_axes.png")
    figure.savefig(output_path, dpi=160)
    print(f"saved: {output_path}")


def plot_signed_orientation_diagnostics(rows, time_s, csv_path):
    teleop_active = values(rows, "teleop_active") > 0.5
    series = {
        "OMY clutch-relative rotation": np.column_stack([
            values(rows, "omy_delta_rotvec_x_rad"),
            values(rows, "omy_delta_rotvec_y_rad"),
            values(rows, "omy_delta_rotvec_z_rad"),
        ]),
        "Mapped FR3 rotation increment": np.column_stack([
            values(rows, "mapped_rotation_delta_x_rad"),
            values(rows, "mapped_rotation_delta_y_rad"),
            values(rows, "mapped_rotation_delta_z_rad"),
        ]),
        "FR3 command rotation from session initial": np.column_stack([
            values(rows, "fr3_command_rotvec_x"),
            values(rows, "fr3_command_rotvec_y"),
            values(rows, "fr3_command_rotvec_z"),
        ]),
        "FR3 target rotation from session initial": np.column_stack([
            values(rows, "fr3_target_rotvec_x"),
            values(rows, "fr3_target_rotvec_y"),
            values(rows, "fr3_target_rotvec_z"),
        ]),
    }

    figure, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    figure.suptitle(
        f"Signed Orientation Axis Diagnostics\n{csv_path.name}"
    )
    labels = ("X", "Y", "Z")
    colors = ("tab:red", "tab:green", "tab:blue")

    for axis, (title, data) in zip(axes.flat, series.items()):
        for index, label in enumerate(labels):
            axis.plot(
                time_s,
                np.rad2deg(data[:, index]),
                color=colors[index],
                label=label,
            )
        shade_inactive_intervals(axis, time_s, teleop_active)
        axis.set_title(title)
        axis.set_ylabel("Rotation vector [deg]")
        axis.grid(True)
        axis.legend()

    for axis in axes[1, :]:
        axis.set_xlabel("Time [s]")

    figure.tight_layout()
    output_path = csv_path.with_name(
        f"{csv_path.stem}_signed_orientation_axes.png"
    )
    figure.savefig(output_path, dpi=160)
    print(f"saved: {output_path}")


def plot_target_acceleration(rows, time_s, csv_path):
    teleop_active = values(rows, "teleop_active") > 0.5
    target_speed = values(rows, "target_linear_speed_mps")
    linear_accel_limit = 2.0
    vector_fields = (
        "target_linear_acceleration_x_mps2",
        "target_linear_acceleration_y_mps2",
        "target_linear_acceleration_z_mps2",
    )
    has_vector_acceleration = all(
        field in rows[0] for field in vector_fields
    )

    figure, axes = plt.subplots(1, 2, figsize=(13, 4), sharex=True)
    figure.suptitle(f"Target Linear Acceleration Limit\n{csv_path.name}")

    axes[0].plot(time_s, target_speed, label="target linear speed")
    shade_inactive_intervals(axes[0], time_s, teleop_active)
    axes[0].set_ylabel("Speed [m/s]")
    axes[0].set_xlabel("Time [s]")
    axes[0].grid(True)
    axes[0].legend()

    if has_vector_acceleration:
        labels = ("X", "Y", "Z")
        colors = ("tab:red", "tab:green", "tab:blue")
        for field, label, color in zip(vector_fields, labels, colors):
            axes[1].plot(
                time_s,
                values(rows, field),
                color=color,
                label=f"target acceleration {label}",
            )
        axes[1].plot(
            time_s,
            values(rows, "target_linear_acceleration_norm_mps2"),
            color="black",
            linewidth=1.2,
            label="target acceleration norm",
        )
    else:
        if "target_linear_acceleration_norm_mps2" in rows[0]:
            axes[1].plot(
                time_s,
                values(rows, "target_linear_acceleration_norm_mps2"),
                color="black",
                label="target acceleration norm",
            )
        else:
            axes[1].plot(
                time_s,
                np.gradient(target_speed, time_s),
                label="estimated target acceleration",
            )
    axes[1].axhline(
        linear_accel_limit,
        color="tab:red",
        linestyle="--",
        label="+MAX_TARGET_LINEAR_ACCEL",
    )
    axes[1].axhline(
        -linear_accel_limit,
        color="tab:red",
        linestyle="--",
        label="-MAX_TARGET_LINEAR_ACCEL",
    )
    shade_inactive_intervals(axes[1], time_s, teleop_active)
    axes[1].set_ylabel("Acceleration [m/s²]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True)
    axes[1].legend()

    figure.tight_layout()
    output_path = csv_path.with_name(
        f"{csv_path.stem}_target_acceleration.png"
    )
    figure.savefig(output_path, dpi=160)
    print(f"saved: {output_path}")


def plot_target_angular_dynamics(rows, time_s, csv_path):
    teleop_active = values(rows, "teleop_active") > 0.5
    angular_speed = values_any(
        rows,
        "target_angular_speed",
        "target_angular_speed_radps",
    )
    acceleration_fields = (
        "target_angular_acceleration_x",
        "target_angular_acceleration_y",
        "target_angular_acceleration_z",
    )
    acceleration_fields_rad = tuple(
        f"{field}_radps2" for field in acceleration_fields
    )
    if all(field in rows[0] for field in acceleration_fields):
        selected_fields = acceleration_fields
        norm_field = "target_angular_acceleration_norm"
    elif all(field in rows[0] for field in acceleration_fields_rad):
        selected_fields = acceleration_fields_rad
        norm_field = "target_angular_acceleration_norm_radps2"
    else:
        selected_fields = None

    figure, axes = plt.subplots(1, 2, figsize=(13, 4), sharex=True)
    figure.suptitle(f"Target Angular Speed and Acceleration\n{csv_path.name}")

    axes[0].plot(time_s, angular_speed, label="target angular speed")
    shade_inactive_intervals(axes[0], time_s, teleop_active)
    axes[0].set_ylabel("Angular speed [rad/s]")
    axes[0].set_xlabel("Time [s]")
    axes[0].grid(True)
    axes[0].legend()

    if selected_fields is not None:
        labels = ("X", "Y", "Z")
        colors = ("tab:red", "tab:green", "tab:blue")
        for field, label, color in zip(selected_fields, labels, colors):
            axes[1].plot(
                time_s,
                values(rows, field),
                color=color,
                label=f"target angular acceleration {label}",
            )
        axes[1].plot(
            time_s,
            values(rows, norm_field),
            color="black",
            linewidth=1.2,
            label="target angular acceleration norm",
        )
    else:
        if "target_angular_acceleration_norm_radps2" in rows[0]:
            axes[1].plot(
                time_s,
                values(rows, "target_angular_acceleration_norm_radps2"),
                color="black",
                label="target angular acceleration norm",
            )
        else:
            axes[1].plot(
                time_s,
                np.gradient(angular_speed, time_s),
                label="estimated angular acceleration",
            )
    axes[1].axhline(
        2.0,
        color="tab:red",
        linestyle="--",
        label="+MAX_TARGET_ANGULAR_ACCEL",
    )
    axes[1].axhline(
        -2.0,
        color="tab:red",
        linestyle="--",
        label="-MAX_TARGET_ANGULAR_ACCEL",
    )
    shade_inactive_intervals(axes[1], time_s, teleop_active)
    axes[1].set_ylabel("Angular acceleration [rad/s²]")
    axes[1].set_xlabel("Time [s]")
    axes[1].grid(True)
    axes[1].legend()

    figure.tight_layout()
    output_path = csv_path.with_name(
        f"{csv_path.stem}_target_angular_dynamics.png"
    )
    figure.savefig(output_path, dpi=160)
    print(f"saved: {output_path}")


def main():
    csv_path = resolve_csv_path()
    png_path = csv_path.with_suffix(".png")
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    rows = load_log(csv_path)
    time_name = "wall_time" if "wall_time" in rows[0] else "timestamp"
    time_s = values(rows, time_name)
    time_s -= time_s[0]

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    fig.suptitle(f"OMY → FR3 Cumulative Clutch Teleoperation\n{csv_path.name}")

    axes[0, 0].plot(
        time_s,
        values_any(rows, "position_error_mm")
        if "position_error_mm" in rows[0]
        else values_any(rows, "position_error_norm") * 1000,
    )
    axes[0, 0].set_ylabel("Position error [mm]")
    axes[0, 0].grid(True)

    omy_rotation_names = (
        "omy_relative_rotation_deg",
        "omy_relative_angle_deg",
    )
    if any(name in rows[0] for name in omy_rotation_names):
        axes[0, 1].plot(
            time_s,
            values_any(rows, *omy_rotation_names),
            label="OMY relative",
        )
    axes[0, 1].plot(
        time_s,
        values_any(rows, "fr3_command_cumulative_rotation_deg")
        if "fr3_command_cumulative_rotation_deg" in rows[0]
        else rotvec_norm_deg(rows, "fr3_command_rotvec"),
        label="FR3 command",
    )
    axes[0, 1].plot(
        time_s,
        values_any(rows, "fr3_target_cumulative_rotation_deg", "fr3_target_angle_deg")
        if any(
            name in rows[0]
            for name in (
                "fr3_target_cumulative_rotation_deg",
                "fr3_target_angle_deg",
            )
        )
        else rotvec_norm_deg(rows, "fr3_target_rotvec"),
        label="FR3 target",
    )
    axes[0, 1].set_ylabel("Cumulative rotation [deg]")
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    axes[1, 0].plot(
        time_s,
        values_any(rows, "command_target_rotation_gap_deg"),
        label="command-target gap",
    )
    axes[1, 0].plot(
        time_s,
        values_any(rows, "orientation_error_deg"),
        label="FR3 tracking error",
    )
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("Rotation error [deg]")
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    axes[1, 1].plot(
        time_s,
        values_any(rows, "raw_max_qdot_radps", "raw_max_dq"),
        label="raw max qdot",
    )
    axes[1, 1].plot(
        time_s,
        values_any(rows, "cmd_max_qdot_radps", "cmd_max_dq"),
        label="command max qdot",
    )
    saturated = values_any(rows, "qdot_saturated", "dq_saturated") > 0.5
    if np.any(saturated):
        axes[1, 1].fill_between(
            time_s,
            0,
            1,
            where=saturated,
            transform=axes[1, 1].get_xaxis_transform(),
            color="tab:red",
            alpha=0.15,
            label="saturated",
        )
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Max joint velocity [rad/s]")
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    fig.tight_layout()
    fig.savefig(png_path, dpi=160)
    print(f"saved: {png_path}")

    if "omy_delta_position_x_m" in rows[0]:
        plot_signed_position_diagnostics(rows, time_s, csv_path)
    if "omy_delta_rotvec_x_rad" in rows[0]:
        plot_signed_orientation_diagnostics(rows, time_s, csv_path)
    plot_target_acceleration(rows, time_s, csv_path)
    plot_target_angular_dynamics(rows, time_s, csv_path)


if __name__ == "__main__":
    main()
