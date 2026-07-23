#!/usr/bin/python3

"""Draft bridge: OMY joint states -> OMY EE delta -> FR3 viewer.

This first draft deliberately does not contain FR3 IK or FR3 joint control.
It is intended to verify that OMY joint states and the OMY EE pose are
calculated correctly while the FR3 model is displayed at its home posture.
"""

import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


ROOT = Path(__file__).resolve().parents[1]
OMY_MODEL_PATH = ROOT / "robotis_mujoco_menagerie" / "robotis_omy" / "scene.xml"
FR3_MODEL_PATH = ROOT / "mujoco_menagerie" / "franka_fr3" / "scene.xml"

OMY_ROS_JOINTS = [f"joint{i}" for i in range(1, 7)]
OMY_MUJOCO_JOINTS = [f"Joint{i}" for i in range(1, 7)]


class OmyPose(Node):
    """Store the latest OMY leader joint positions received from ROS."""

    def __init__(self, target_positions, target_lock):
        super().__init__("fr3_omy_bridge")
        self.target_positions = target_positions
        self.target_lock = target_lock
        self.last_message_time = 0.0

        self.subscription = self.create_subscription(
            JointState,
            "/leader/joint_states",
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, message):
        received = dict(zip(message.name, message.position))

        with self.target_lock:
            for index, joint_name in enumerate(OMY_ROS_JOINTS):
                if joint_name in received:
                    self.target_positions[index] = received[joint_name]
            self.last_message_time = time.monotonic()


def keyframe_id(model, name):
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
    if key_id < 0:
        raise ValueError(f"keyframe not found: {name}")
    return key_id


def site_id(model, name):
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if site < 0:
        raise ValueError(f"site not found: {name}")
    return site


def read_site_pose(data, site):
    position = data.site_xpos[site].copy()
    rotation = data.site_xmat[site].reshape(3, 3).copy()
    return position, rotation


def rotation_error(current_rotation, target_rotation):
    """Return a small-angle rotation error in the world frame."""
    return 0.5 * (
        np.cross(current_rotation[:, 0], target_rotation[:, 0])
        + np.cross(current_rotation[:, 1], target_rotation[:, 1])
        + np.cross(current_rotation[:, 2], target_rotation[:, 2])
    )


def solve_dls_ik(
    model,
    data,
    site,
    q_seed,
    target_position,
    target_rotation,
    qpos_addresses,
    dof_addresses,
    joint_limits,
    iterations=10,
    damping=0.05,
):
    """Solve a 6D site pose with damped-least-squares IK."""
    q_solution = q_seed.copy()

    for _ in range(iterations):
        data.qpos[qpos_addresses] = q_solution
        mujoco.mj_forward(model, data)

        current_position, current_rotation = read_site_pose(data, site)
        position_error = target_position - current_position
        orientation_error = rotation_error(current_rotation, target_rotation)
        pose_error = np.concatenate((position_error, orientation_error))

        if np.linalg.norm(pose_error) < 1e-4:
            break

        jac_position = np.zeros((3, model.nv))
        jac_rotation = np.zeros((3, model.nv))
        mujoco.mj_jacSite(
            model,
            data,
            jac_position,
            jac_rotation,
            site,
        )
        jacobian = np.vstack((jac_position, jac_rotation))[:, dof_addresses]

        identity = np.eye(6)
        dq = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping**2 * identity,
            pose_error,
        )

        # Limit each iteration so a sudden leader message cannot cause a jump.
        dq_norm = np.linalg.norm(dq)
        if dq_norm > 0.08:
            dq *= 0.08 / dq_norm

        q_solution += 0.8 * dq
        q_solution = np.clip(
            q_solution,
            joint_limits[:, 0],
            joint_limits[:, 1],
        )

    return q_solution


def main():
    rclpy.init()

    # OMY is used for forward kinematics only. Its viewer is not launched.
    omy_model = mujoco.MjModel.from_xml_path(str(OMY_MODEL_PATH))
    omy_data = mujoco.MjData(omy_model)
    omy_home_id = keyframe_id(omy_model, "home")
    mujoco.mj_resetDataKeyframe(omy_model, omy_data, omy_home_id)
    mujoco.mj_forward(omy_model, omy_data)
    omy_ee_site_id = site_id(omy_model, "omy_ee_site")
    omy_home_position, omy_home_rotation = read_site_pose(
        omy_data, omy_ee_site_id
    )

    # FR3 is the model shown in the MuJoCo viewer.
    fr3_model = mujoco.MjModel.from_xml_path(str(FR3_MODEL_PATH))
    fr3_data = mujoco.MjData(fr3_model)
    fr3_home_id = keyframe_id(fr3_model, "home")
    mujoco.mj_resetDataKeyframe(fr3_model, fr3_data, fr3_home_id)
    mujoco.mj_forward(fr3_model, fr3_data)
    fr3_ee_site_id = site_id(fr3_model, "attachment_site")
    fr3_home_position, fr3_home_rotation = read_site_pose(
        fr3_data, fr3_ee_site_id
    )

    fr3_joint_ids = [
        fr3_model.joint(f"fr3_joint{i}").id
        for i in range(1, 8)
    ]
    fr3_qpos_addresses = np.array(
        [fr3_model.jnt_qposadr[joint_id] for joint_id in fr3_joint_ids],
        dtype=int,
    )
    fr3_dof_addresses = np.array(
        [fr3_model.jnt_dofadr[joint_id] for joint_id in fr3_joint_ids],
        dtype=int,
    )
    fr3_joint_limits = fr3_model.jnt_range[fr3_joint_ids].copy()
    fr3_actuator_ids = np.array(
        [fr3_model.actuator(f"fr3_joint{i}").id for i in range(1, 8)],
        dtype=int,
    )
    fr3_ik_data = mujoco.MjData(fr3_model)

    omy_joint_qpos_addresses = [
        omy_model.jnt_qposadr[omy_model.joint(name).id]
        for name in OMY_MUJOCO_JOINTS
    ]

    target_positions = omy_data.qpos[omy_joint_qpos_addresses].copy()
    target_lock = threading.Lock()
    ros_node = OmyPose(target_positions, target_lock)
    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True,
    )
    ros_thread.start()

    print("OMY home position:", omy_home_position)
    print("FR3 home position:", fr3_home_position)
    print("Waiting for /leader/joint_states...")

    last_print_time = 0.0

    try:
        with mujoco.viewer.launch_passive(fr3_model, fr3_data) as viewer:
            while viewer.is_running() and rclpy.ok():
                with target_lock:
                    omy_target = target_positions.copy()

                for address, position in zip(omy_joint_qpos_addresses, omy_target):
                    omy_data.qpos[address] = position
                mujoco.mj_forward(omy_model, omy_data)

                omy_current_position, omy_current_rotation = read_site_pose(
                    omy_data, omy_ee_site_id
                )

                # Body-frame pose increment from OMY home to current pose.
                delta_position = omy_current_position - omy_home_position
                delta_rotation = omy_home_rotation.T @ omy_current_rotation

                # Apply OMY's home-relative motion to the FR3 home pose.
                fr3_target_position = fr3_home_position + delta_position
                fr3_target_rotation = fr3_home_rotation @ delta_rotation

                # Solve FR3's 7-joint configuration for the target EE pose.
                fr3_ik_data.qpos[:] = fr3_data.qpos
                fr3_ik_qpos = fr3_ik_data.qpos[fr3_qpos_addresses].copy()
                fr3_ik_qpos = solve_dls_ik(
                    fr3_model,
                    fr3_ik_data,
                    fr3_ee_site_id,
                    fr3_ik_qpos,
                    fr3_target_position,
                    fr3_target_rotation,
                    fr3_qpos_addresses,
                    fr3_dof_addresses,
                    fr3_joint_limits,
                )

                fr3_data.ctrl[fr3_actuator_ids] = fr3_ik_qpos
                mujoco.mj_step(fr3_model, fr3_data)

                now = time.monotonic()
                if now - last_print_time >= 0.2:
                    print("OMY current position:", omy_current_position)
                    print("OMY delta position:", delta_position)
                    print("OMY delta rotation:\n", delta_rotation)
                    print("FR3 target position:", fr3_target_position)
                    print("FR3 IK qpos:", fr3_ik_qpos)
                    last_print_time = now

                viewer.sync()
                time.sleep(0.002)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
