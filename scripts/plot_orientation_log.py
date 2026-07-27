#!/usr/bin/env python3

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = ROOT / "logs" / "MAX_DQ_0.004_v2.csv"
PNG_PATH = ROOT / "logs" / "MAX_DQ_0.004_v2.png"


def load_log(path):
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV is empty: {path}")
    return rows


def values(rows, name):
    return np.array([float(row[name]) for row in rows])


def main():
    if not CSV_PATH.exists():
        raise FileNotFoundError(f"CSV not found: {CSV_PATH}")

    rows = load_log(CSV_PATH)
    time_s = values(rows, "timestamp")
    time_s -= time_s[0]

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True)
    fig.suptitle("OMY → FR3 Orientation Teleoperation")

    axes[0, 0].plot(time_s, values(rows, "position_error_norm") * 1000)
    axes[0, 0].set_ylabel("Position error [mm]")
    axes[0, 0].grid(True)

    axes[0, 1].plot(time_s, values(rows, "omy_relative_angle_deg"), label="OMY")
    axes[0, 1].plot(time_s, values(rows, "fr3_target_angle_deg"), label="FR3 target")
    axes[0, 1].plot(time_s, values(rows, "fr3_actual_angle_deg"), label="FR3 actual")
    axes[0, 1].set_ylabel("Rotation from initial [deg]")
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    axes[1, 0].plot(time_s, values(rows, "orientation_error_deg"))
    axes[1, 0].set_xlabel("Time [s]")
    axes[1, 0].set_ylabel("Orientation tracking error [deg]")
    axes[1, 0].grid(True)

    axes[1, 1].plot(time_s, values(rows, "max_joint_step"))
    axes[1, 1].set_xlabel("Time [s]")
    axes[1, 1].set_ylabel("Max joint step [rad]")
    axes[1, 1].grid(True)

    fig.tight_layout()
    fig.savefig(PNG_PATH, dpi=160)
    print(f"saved: {PNG_PATH}")


if __name__ == "__main__":
    main()
