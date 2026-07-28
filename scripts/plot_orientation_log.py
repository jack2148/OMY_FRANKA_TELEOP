#!/usr/bin/env python3

import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CSV_PATH = ROOT / "logs" / "refactored_teleop_20260728_154246.csv"


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


def resolve_csv_path():
    if len(sys.argv) > 1:
        return Path(sys.argv[1]).expanduser().resolve()
    candidates = sorted(ROOT.joinpath("logs").glob("refactored_teleop_*.csv"))
    if candidates:
        return candidates[-1]
    return DEFAULT_CSV_PATH


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

    axes[0, 1].plot(
        time_s,
        values_any(
            rows,
            "omy_relative_rotation_deg",
            "omy_relative_angle_deg",
        ),
        label="OMY relative",
    )
    axes[0, 1].plot(
        time_s,
        values_any(rows, "fr3_command_cumulative_rotation_deg"),
        label="FR3 command",
    )
    axes[0, 1].plot(
        time_s,
        values_any(
            rows,
            "fr3_target_cumulative_rotation_deg",
            "fr3_target_angle_deg",
        ),
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


if __name__ == "__main__":
    main()
