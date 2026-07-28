#!/usr/bin/env python3

"""Plot both legacy and refactored teleoperation CSV logs."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT / "logs"


def choose_csv(requested):
    if requested:
        path = Path(requested)
        return path if path.is_absolute() else ROOT / path

    patterns = (
        "base_frame_cumulative_*.csv",
        "refactored_teleop_*.csv",
        "target_anchor_*.csv",
        "MAX_DQ_0.004_v2.csv",
        "orientation_teleop.csv",
    )
    candidates = []
    for pattern in patterns:
        candidates.extend(LOG_DIR.glob(pattern))
    if not candidates:
        raise FileNotFoundError(f"No CSV logs found in {LOG_DIR}")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def load_rows(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV is empty: {path}")
    return rows


def column(rows, *names):
    for name in names:
        if name in rows[0]:
            return name
    return None


def series(rows, name):
    return np.asarray([float(row[name]) for row in rows], dtype=float)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv_path",
        nargs="?",
        help="CSV path, relative to the repository or absolute",
    )
    parser.add_argument(
        "--output",
        help="Optional output PNG path; defaults to the CSV stem",
    )
    args = parser.parse_args()

    csv_path = choose_csv(args.csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"CSV not found: {csv_path}")

    rows = load_rows(csv_path)
    time_name = column(rows, "wall_time", "timestamp", "sim_time")
    if time_name is None:
        raise KeyError("CSV must contain wall_time, timestamp, or sim_time")
    time_s = series(rows, time_name)
    time_s -= time_s[0]

    position_name = column(rows, "position_error_mm", "position_error_norm")
    if position_name is None:
        raise KeyError("CSV has no position error column")
    position_error = series(rows, position_name)
    if position_name == "position_error_norm":
        position_error *= 1000.0

    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex=True)
    fig.suptitle(f"OMY → FR3 Teleoperation — {csv_path.name}")

    # 1. Cartesian position tracking.
    axes[0, 0].plot(time_s, position_error, color="tab:blue")
    axes[0, 0].set_ylabel("Position error [mm]")
    axes[0, 0].grid(True)

    # 2. Orientation trajectory if legacy columns exist; otherwise show the
    # refactored target angular speed, which is the conditioning signal.
    orientation_names = (
        "omy_relative_angle_deg",
        "fr3_target_angle_deg",
        "fr3_actual_angle_deg",
    )
    if all(name in rows[0] for name in orientation_names):
        for name, label in zip(
            orientation_names,
            ("OMY", "FR3 target", "FR3 actual"),
        ):
            axes[0, 1].plot(time_s, series(rows, name), label=label)
        axes[0, 1].set_ylabel("Rotation from anchor [deg]")
        axes[0, 1].legend()
    elif all(
        name in rows[0]
        for name in (
            "omy_delta_rotation_deg",
            "fr3_command_cumulative_rotation_deg",
            "fr3_target_cumulative_rotation_deg",
        )
    ):
        axes[0, 1].plot(
            time_s,
            series(rows, "omy_delta_rotation_deg"),
            label="OMY stroke",
        )
        axes[0, 1].plot(
            time_s,
            series(rows, "fr3_command_cumulative_rotation_deg"),
            label="FR3 command",
        )
        axes[0, 1].plot(
            time_s,
            series(rows, "fr3_target_cumulative_rotation_deg"),
            label="FR3 target",
        )
        axes[0, 1].set_ylabel("Rotation from session start [deg]")
        axes[0, 1].legend()
    elif "target_angular_speed_radps" in rows[0]:
        axes[0, 1].plot(
            time_s,
            series(rows, "target_angular_speed_radps"),
            label="Target angular speed",
            color="tab:orange",
        )
        axes[0, 1].set_ylabel("Target angular speed [rad/s]")
        axes[0, 1].legend()
    else:
        axes[0, 1].text(0.5, 0.5, "No orientation trajectory column", ha="center")
        axes[0, 1].set_ylabel("Orientation")
    axes[0, 1].grid(True)

    # 3. Orientation tracking error.
    if "orientation_error_deg" in rows[0]:
        axes[1, 0].plot(time_s, series(rows, "orientation_error_deg"))
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("Orientation error [deg]")
    axes[1, 0].grid(True)

    # 4. Raw/commanded qdot and saturation.
    raw_name = column(rows, "raw_max_qdot_radps", "raw_max_dq")
    cmd_name = column(rows, "cmd_max_qdot_radps", "cmd_max_dq", "max_joint_step")
    if raw_name:
        axes[1, 1].plot(time_s, series(rows, raw_name), label="raw", alpha=0.8)
    if cmd_name:
        axes[1, 1].plot(time_s, series(rows, cmd_name), label="command", alpha=0.8)
    saturated_name = column(rows, "qdot_saturated", "dq_saturated")
    if saturated_name:
        saturated = series(rows, saturated_name).astype(bool)
        if np.any(saturated):
            axes[1, 1].scatter(
                time_s[saturated],
                series(rows, cmd_name)[saturated],
                s=8,
                color="red",
                label="saturated",
            )
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Joint speed [rad/s] or step [rad]")
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    fig.tight_layout()
    output = Path(args.output) if args.output else csv_path.with_suffix(".png")
    if not output.is_absolute():
        output = ROOT / output
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    print(f"CSV: {csv_path}")
    print(f"PNG: {output}")


if __name__ == "__main__":
    main()
