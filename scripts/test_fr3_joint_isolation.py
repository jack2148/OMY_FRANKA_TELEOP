#!/usr/bin/env python3
"""Move one MuJoCo FR3 joint at a time for joint-axis inspection."""

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = ROOT / "mujoco_menagerie" / "franka_fr3" / "scene.xml"
CONTROL_HZ = 1000.0


def main():
    parser = argparse.ArgumentParser(
        description="Isolate one FR3 joint in MuJoCo."
    )
    parser.add_argument(
        "--joint",
        type=int,
        choices=range(1, 8),
        required=True,
        help="FR3 joint number to move: 1 through 7",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=0.25,
        help="sinusoidal joint amplitude in radians",
    )
    parser.add_argument(
        "--frequency",
        type=float,
        default=0.20,
        help="sinusoidal motion frequency in Hz",
    )
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    model.opt.timestep = 1.0 / CONTROL_HZ

    joint_names = [f"fr3_joint{i}" for i in range(1, 8)]
    joint_ids = np.array(
        [model.joint(name).id for name in joint_names],
        dtype=int,
    )
    qpos_indices = model.jnt_qposadr[joint_ids]
    actuator_ids = np.array(
        [model.actuator(name).id for name in joint_names],
        dtype=int,
    )

    q_home = data.qpos[qpos_indices].copy()
    selected = args.joint - 1
    joint_id = joint_ids[selected]
    joint_axis = model.jnt_axis[joint_id].copy()
    joint_range = model.jnt_range[joint_id].copy()

    data.ctrl[actuator_ids] = q_home
    mujoco.mj_forward(model, data)

    print(f"model: {MODEL_PATH}")
    print(f"moving joint: {joint_names[selected]}")
    print(f"MuJoCo joint axis: {joint_axis}")
    print(f"joint range: {joint_range}")
    print("All other FR3 joints are held at the initial qpos.")
    print("Close the viewer or press Ctrl-C to stop.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start = time.perf_counter()
        next_tick = start
        while viewer.is_running():
            now = time.perf_counter()
            elapsed = now - start
            q_target = q_home.copy()
            q_target[selected] = (
                q_home[selected]
                + args.amplitude
                * np.sin(2.0 * np.pi * args.frequency * elapsed)
            )
            q_target[selected] = np.clip(
                q_target[selected],
                joint_range[0],
                joint_range[1],
            )

            data.ctrl[actuator_ids] = q_target
            mujoco.mj_step(model, data)
            viewer.sync()

            next_tick += model.opt.timestep
            remaining = next_tick - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_tick = time.perf_counter()


if __name__ == "__main__":
    main()
