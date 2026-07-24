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
TRIGGER_JOINT = "rh_r1_joint"
# Inward pull is assumed to move rh_r1_joint toward the lower position.
# Hysteresis: lower value turns position mode on, higher value turns it off.
TRIGGER_ON_THRESHOLD = -0.9
TRIGGER_OFF_THRESHOLD = -0.7

# Position-only DLS IK. Orientation tracking remains disabled.
AXIS_MAP = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])
POSITION_SCALE = 0.2
IK_DAMPING = 0.05
IK_GAIN = 0.05
MAX_DQ = 0.002


class OmyPose(Node):
    """Store the latest OMY leader joint positions received from ROS."""

    def __init__(self, target_positions, target_lock):
        super().__init__("fr3_omy_bridge")
        self.target_positions = target_positions
        self.target_lock = target_lock
        self.last_message_time = 0.0
        self.now_joint_state = False
        self.trigger_position = 0.0

        self.subscription = self.create_subscription(
            JointState,
            "/leader/joint_states",
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, message):
        received = dict(zip(message.name, message.position))

        required_joints = OMY_ROS_JOINTS + [TRIGGER_JOINT]
        if not all(name in received for name in required_joints):
            return

        with self.target_lock:
            for index, joint_name in enumerate(OMY_ROS_JOINTS):
                self.target_positions[index] = received[joint_name]

            self.trigger_position = received[TRIGGER_JOINT]
            self.last_message_time = time.monotonic()
            self.now_joint_state = True


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


def draw_target_marker(viewer, position):
    """Draw one red sphere in the viewer's user scene."""
    scene = viewer.user_scn
    scene.ngeom = 1
    mujoco.mjv_initGeom(
        scene.geoms[0],
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([0.025, 0.0, 0.0]),
        position,
        np.eye(3).reshape(-1),
        np.array([1.0, 0.0, 0.0, 1.0]),
    )


def main():
    rclpy.init()

    # OMY is used for forward kinematics only. Its viewer is not launched.
    omy_model = mujoco.MjModel.from_xml_path(str(OMY_MODEL_PATH))
    omy_data = mujoco.MjData(omy_model)
    omy_home_id = keyframe_id(omy_model, "home")
    mujoco.mj_resetDataKeyframe(omy_model, omy_data, omy_home_id)
    mujoco.mj_forward(omy_model, omy_data)
    omy_ee_site_id = site_id(omy_model, "omy_ee_site")
    omy_home_position, _ = read_site_pose(
        omy_data, omy_ee_site_id
    )

    # FR3 is the model shown in the MuJoCo viewer.
    fr3_model = mujoco.MjModel.from_xml_path(str(FR3_MODEL_PATH))
    fr3_data = mujoco.MjData(fr3_model)
    fr3_home_id = keyframe_id(fr3_model, "home")
    mujoco.mj_resetDataKeyframe(fr3_model, fr3_data, fr3_home_id)
    mujoco.mj_forward(fr3_model, fr3_data)
    fr3_ee_site_id = site_id(fr3_model, "attachment_site")
    fr3_home_position, _ = read_site_pose(
        fr3_data, fr3_ee_site_id
    )

    fr3_joint_ids = np.array(
        [fr3_model.joint(f"fr3_joint{i}").id for i in range(1, 8)],
        dtype=int,
    )
    fr3_qpos_indices = np.array(
        [fr3_model.jnt_qposadr[joint_id] for joint_id in fr3_joint_ids],
        dtype=int,
    )
    fr3_dof_indices = np.array(
        [fr3_model.jnt_dofadr[joint_id] for joint_id in fr3_joint_ids],
        dtype=int,
    )
    fr3_joint_lower = fr3_model.jnt_range[fr3_joint_ids, 0].copy()
    fr3_joint_upper = fr3_model.jnt_range[fr3_joint_ids, 1].copy()
    fr3_actuator_indices = np.array(
        [fr3_model.actuator(f"fr3_joint{i}").id for i in range(1, 8)],
        dtype=int,
    )
    fr3_home_q = fr3_data.qpos[fr3_qpos_indices].copy()
    fr3_data.qpos[fr3_qpos_indices] = fr3_home_q.copy()
    fr3_data.ctrl[fr3_actuator_indices] = fr3_home_q.copy()
    mujoco.mj_forward(fr3_model, fr3_data)

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

    # XML home initializes the displayed FR3. The actual reference is captured
    # on each OFF -> ON trigger edge.
    position_mode_active = False
    omy_initial_position = None
    fr3_initial_position = None
    fr3_target_position = fr3_home_position.copy()
    fr3_q_command = fr3_home_q.copy()

    try:
        with mujoco.viewer.launch_passive(fr3_model, fr3_data) as viewer:
            while viewer.is_running() and rclpy.ok():
                with target_lock:
                    omy_target = target_positions.copy()
                    trigger_position = ros_node.trigger_position

                for address, position in zip(omy_joint_qpos_addresses, omy_target):
                    omy_data.qpos[address] = position
                mujoco.mj_forward(omy_model, omy_data)

                omy_current_position, _ = read_site_pose(
                    omy_data, omy_ee_site_id
                )
                if not ros_node.now_joint_state:
                    fr3_data.ctrl[fr3_actuator_indices] = fr3_q_command
                    mujoco.mj_step(fr3_model, fr3_data)
                    viewer.sync()
                    time.sleep(0.002)
                    continue

                if (
                    not position_mode_active
                    and trigger_position <= TRIGGER_ON_THRESHOLD
                ):
                    # Capture a new clutch/reference pose exactly when the
                    # position mode is enabled. Keep it fixed while ON.
                    omy_initial_position = omy_current_position.copy()
                    fr3_q_command = fr3_data.ctrl[fr3_actuator_indices].copy()
                    fr3_initial_position, _ = read_site_pose(
                        fr3_data, fr3_ee_site_id
                    )
                    position_mode_active = True

                    print("Position mode ON")
                    print("Runtime initial OMY EE pose captured")
                    print("Runtime initial FR3 EE pose captured")

                elif (
                    position_mode_active
                    and trigger_position >= TRIGGER_OFF_THRESHOLD
                ):
                    position_mode_active = False
                    print("Position mode OFF - holding last FR3 target")

                if not position_mode_active:
                    fr3_data.ctrl[fr3_actuator_indices] = fr3_q_command
                    mujoco.mj_step(fr3_model, fr3_data)
                    viewer.sync()
                    time.sleep(0.002)
                    continue

                # Keep the validated position target calculation unchanged.
                delta_position_omy = omy_current_position - omy_initial_position
                delta_position_fr3 = POSITION_SCALE * (
                    AXIS_MAP @ delta_position_omy
                )
                fr3_target_position = fr3_initial_position + delta_position_fr3
                draw_target_marker(viewer, fr3_target_position)

                # Position-only DLS IK. Orientation is intentionally ignored.
                fr3_current_position = fr3_data.site_xpos[fr3_ee_site_id].copy()
                position_error = fr3_target_position - fr3_current_position

                jacp = np.zeros((3, fr3_model.nv))
                jacr = np.zeros((3, fr3_model.nv))
                mujoco.mj_jacSite(
                    fr3_model,
                    fr3_data,
                    jacp,
                    jacr,
                    fr3_ee_site_id,
                )
                J_pos = jacp[:, fr3_dof_indices]

                regularized = (
                    J_pos @ J_pos.T
                    + (IK_DAMPING ** 2) * np.eye(3)
                )
                dq = J_pos.T @ np.linalg.solve(
                    regularized,
                    position_error,
                )
                dq = IK_GAIN * dq
                dq = np.clip(dq, -MAX_DQ, MAX_DQ)
                max_joint_step = np.max(np.abs(dq))

                q_current = fr3_data.qpos[fr3_qpos_indices].copy()
                fr3_q_command = np.clip(
                    fr3_q_command + dq,
                    fr3_joint_lower,
                    fr3_joint_upper,
                )
                fr3_data.ctrl[fr3_actuator_indices] = fr3_q_command
                mujoco.mj_step(fr3_model, fr3_data)

                now = time.monotonic()
                if now - last_print_time >= 0.2:
                    tracking_error = np.linalg.norm(position_error)
                    return_error = np.linalg.norm(
                        fr3_current_position - fr3_initial_position
                    )
                    print("OMY current position:", omy_current_position)
                    print("OMY delta position:", delta_position_omy)
                    print("FR3 mapped delta position:", delta_position_fr3)
                    print("Trigger position:", trigger_position)
                    print("Position mode active:", position_mode_active)
                    print("FR3 target position:", fr3_target_position)
                    print("FR3 current position:", fr3_current_position)
                    print("Position error:", position_error)
                    print("FR3 q current:", q_current)
                    print("FR3 q command:", fr3_q_command)
                    print(f"tracking error: {tracking_error * 1000:.2f} mm")
                    print(f"return error: {return_error * 1000:.2f} mm")
                    print(f"max joint step: {max_joint_step:.6f} rad")
                    last_print_time = now

                viewer.sync()
                time.sleep(0.002)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
