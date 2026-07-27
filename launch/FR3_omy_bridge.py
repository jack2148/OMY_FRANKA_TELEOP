#!/usr/bin/python3

"""Draft bridge: OMY joint states -> OMY EE delta -> FR3 viewer.

This first draft deliberately does not contain FR3 IK or FR3 joint control.
It is intended to verify that OMY joint states and the OMY EE pose are
calculated correctly while the FR3 model is displayed at its home posture.
"""

import csv
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
# Temporarily disabled while focusing on runtime behavior.
ENABLE_CSV_LOGGING = False

OMY_ROS_JOINTS = [f"joint{i}" for i in range(1, 7)]
OMY_MUJOCO_JOINTS = [f"Joint{i}" for i in range(1, 7)]
TRIGGER_JOINT = "rh_r1_joint"
# Inward pull is assumed to move rh_r1_joint toward the lower position.
# Hysteresis: lower value turns position mode on, higher value turns it off.
TRIGGER_ON_THRESHOLD = -0.9
TRIGGER_OFF_THRESHOLD = -0.7

# Position mapping is validated. Orientation mapping is still a candidate and
# must be checked with the marker before it is used by an IK objective.
AXIS_MAP = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])
# Candidate orientation mapping. Keep this separate from the validated
# position mapping so it can be changed independently after one-axis tests.
R_FR3_FROM_OMY_ORIENTATION = AXIS_MAP.copy()
# Orientation-only direction adjustment: keep roll/pitch, reverse yaw.
# Order is [roll, pitch, yaw].
ORIENTATION_RPY_SIGN = np.array([1.0, -1.0, -1.0])
POSITION_SCALE = 0.7
IK_DAMPING = 0.05
IK_GAIN = 0.05
# Relative scale between rotational error (rad) and position error (m) in
# the combined task. Tune this after checking the marker and one-axis tests.
ROTATION_IK_WEIGHT = 0.1
MAX_DQ = 0.004


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


def rotation_matrix_to_rotvec(rotation):
    """Convert a proper 3x3 rotation matrix to an axis-angle vector."""
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = np.arccos(cosine)
    if angle < 1e-7:
        return 0.5 * np.array([
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ])

    if np.pi - angle < 1e-5:
        # The skew formula becomes ill-conditioned near pi.
        axis = np.sqrt(np.maximum(np.diag(rotation) + 1.0, 0.0) * 0.5)
        major = int(np.argmax(axis))
        if axis[major] < 1e-7:
            return np.zeros(3)
        for index in range(3):
            if index != major:
                axis[index] = (
                    rotation[major, index] + rotation[index, major]
                ) / (4.0 * axis[major])
        return angle * (axis / np.linalg.norm(axis))

    vee = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    return angle * vee / (2.0 * np.sin(angle))


def rotation_matrix_to_angle_deg(rotation):
    """Return the equivalent axis-angle magnitude in degrees."""
    return float(np.degrees(np.linalg.norm(rotation_matrix_to_rotvec(rotation))))


def rotation_matrix_to_rotvec_deg(rotation):
    """Return the axis-angle rotation vector in degrees."""
    return np.degrees(rotation_matrix_to_rotvec(rotation))


def print_orientation_debug(
    omy_initial_rotation,
    omy_current_rotation,
    fr3_initial_rotation,
    fr3_target_rotation,
    fr3_current_rotation,
):
    """Print all rotations relative to the same runtime initial pose."""
    omy_relative = omy_initial_rotation.T @ omy_current_rotation
    target_relative = fr3_initial_rotation.T @ fr3_target_rotation
    actual_relative = fr3_initial_rotation.T @ fr3_current_rotation
    tracking_error = fr3_target_rotation @ fr3_current_rotation.T

    print(
        "\n[Orientation]"
        f"\n  OMY relative   : {rotation_matrix_to_angle_deg(omy_relative):7.3f} deg,"
        f" rotvec={np.round(rotation_matrix_to_rotvec_deg(omy_relative), 3)}"
        f"\n  FR3 target     : {rotation_matrix_to_angle_deg(target_relative):7.3f} deg,"
        f" rotvec={np.round(rotation_matrix_to_rotvec_deg(target_relative), 3)}"
        f"\n  FR3 actual     : {rotation_matrix_to_angle_deg(actual_relative):7.3f} deg,"
        f" rotvec={np.round(rotation_matrix_to_rotvec_deg(actual_relative), 3)}"
        f"\n  Tracking error : {rotation_matrix_to_angle_deg(tracking_error):7.3f} deg"
    )


def rotation_matrix_to_rpy(rotation):
    """Return intrinsic XYZ roll, pitch, yaw in radians (ZYX extraction)."""
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def rpy_to_rotation_matrix(rpy):
    """Build a rotation matrix from intrinsic XYZ roll, pitch, yaw."""
    roll, pitch, yaw = rpy
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
        [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
        [-sy, cy * sx, cy * cx],
    ])


def draw_target_marker(viewer, position, rotation):
    """Draw a red target sphere and RGB orientation triad."""
    scene = viewer.user_scn
    scene.ngeom = 4
    mujoco.mjv_initGeom(
        scene.geoms[0],
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([0.025, 0.0, 0.0]),
        position,
        np.eye(3).reshape(-1),
        np.array([1.0, 0.0, 0.0, 1.0]),
    )

    axis_colors = (
        np.array([1.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0, 1.0]),
        np.array([0.0, 0.0, 1.0, 1.0]),
    )
    marker_length = 0.045
    marker_radius = 0.004
    for geom_index, axis_index in enumerate(range(3), start=1):
        axis = rotation[:, axis_index]
        # A box's local z axis is aligned with the target frame axis.
        other_a = rotation[:, (axis_index + 1) % 3]
        other_b = rotation[:, (axis_index + 2) % 3]
        geom_rotation = np.column_stack((other_a, other_b, axis))
        geom_position = position + 0.5 * marker_length * axis
        mujoco.mjv_initGeom(
            scene.geoms[geom_index],
            mujoco.mjtGeom.mjGEOM_BOX,
            np.array([marker_radius, marker_radius, 0.5 * marker_length]),
            geom_position,
            geom_rotation.reshape(-1),
            axis_colors[axis_index],
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

    #FR3 is the model shown in the MuJoCo viewer.
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
    last_loop_time = time.monotonic()
    log_step = 0
    return_near_omy_home = False

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
                loop_time = time.monotonic()
                dt = loop_time - last_loop_time
                last_loop_time = loop_time

                with target_lock:
                    omy_target = target_positions.copy()
                    trigger_position = ros_node.trigger_position

                for address, position in zip(omy_joint_qpos_addresses, omy_target):
                    omy_data.qpos[address] = position
                mujoco.mj_forward(omy_model, omy_data)

                omy_current_position, omy_current_rotation = read_site_pose(
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
                    omy_initial_rotation = omy_current_rotation.copy()
                    fr3_q_command = fr3_data.ctrl[fr3_actuator_indices].copy()
                    fr3_initial_position, fr3_initial_rotation = read_site_pose(
                        fr3_data, fr3_ee_site_id
                    )
                    position_mode_active = True

                    omy_delta_rotation = (
                        omy_initial_rotation.T @ omy_current_rotation
                    )

                    print("R_omy_delta at trigger ON:", omy_delta_rotation)
                    print("det:", np.linalg.det(omy_delta_rotation))
                    print(
                        "orthogonality error:",
                        np.linalg.norm(
                            omy_delta_rotation.T @ omy_delta_rotation
                            - np.eye(3)
                        ),
                    )

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
                omy_delta_rotation = (
                    omy_initial_rotation.T @ omy_current_rotation
                )
                omy_delta_rpy = rotation_matrix_to_rpy(omy_delta_rotation)
                omy_delta_rotation_for_target = rpy_to_rotation_matrix(
                    ORIENTATION_RPY_SIGN * omy_delta_rpy
                )
                fr3_delta_rotation = (
                    R_FR3_FROM_OMY_ORIENTATION
                    @ omy_delta_rotation_for_target
                    @ R_FR3_FROM_OMY_ORIENTATION.T
                )
                fr3_target_rotation = fr3_initial_rotation @ fr3_delta_rotation
                draw_target_marker(
                    viewer,
                    fr3_target_position,
                    fr3_target_rotation,
                )

                # Combined position + orientation DLS IK. The orientation
                # error is expressed in the world frame, matching MuJoCo's
                # site angular Jacobian convention.
                fr3_current_position, fr3_current_rotation = read_site_pose(
                    fr3_data, fr3_ee_site_id
                )
                position_error = fr3_target_position - fr3_current_position
                rotation_error_matrix = (
                    fr3_target_rotation @ fr3_current_rotation.T
                )
                rotation_error = rotation_matrix_to_rotvec(
                    rotation_error_matrix
                )
                omy_rotvec = rotation_matrix_to_rotvec(omy_delta_rotation)

                omy_return_pos_error = np.linalg.norm(delta_position_omy)
                omy_return_rot_error = np.linalg.norm(omy_rotvec)
                is_near_omy_home = (
                    omy_return_pos_error < 0.005
                    and omy_return_rot_error < np.deg2rad(2.0)
                )
                if is_near_omy_home and not return_near_omy_home:
                    fr3_return_pos_error = np.linalg.norm(
                        fr3_current_position - fr3_initial_position
                    )
                    fr3_return_rotation_error = (
                        fr3_initial_rotation @ fr3_current_rotation.T
                    )
                    fr3_return_rot_error_deg = rotation_matrix_to_angle_deg(
                        fr3_return_rotation_error
                    )
                    print(
                        f"FR3 return position error: "
                        f"{fr3_return_pos_error * 1000:.2f} mm"
                    )
                    print(
                        f"FR3 return orientation error: "
                        f"{fr3_return_rot_error_deg:.2f} deg"
                    )
                return_near_omy_home = is_near_omy_home

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
                J_rot = jacr[:, fr3_dof_indices]
                task_jacobian = np.vstack((
                    J_pos,
                    ROTATION_IK_WEIGHT * J_rot,
                ))
                task_error = np.concatenate((
                    position_error,
                    ROTATION_IK_WEIGHT * rotation_error,
                ))

                regularized = (
                    task_jacobian @ task_jacobian.T
                    + (IK_DAMPING ** 2) * np.eye(6)
                )
                dq_raw = task_jacobian.T @ np.linalg.solve(
                    regularized,
                    task_error,
                )
                dq_raw = IK_GAIN * dq_raw
                raw_max_dq = np.max(np.abs(dq_raw))
                dq_saturated = bool(raw_max_dq > MAX_DQ)
                dq_cmd = np.clip(dq_raw, -MAX_DQ, MAX_DQ)
                cmd_max_dq = np.max(np.abs(dq_cmd))
                max_joint_step = cmd_max_dq

                q_current = fr3_data.qpos[fr3_qpos_indices].copy()
                fr3_q_command = np.clip(
                    fr3_q_command + dq_cmd,
                    fr3_joint_lower,
                    fr3_joint_upper,
                )
                fr3_data.ctrl[fr3_actuator_indices] = fr3_q_command
                mujoco.mj_step(fr3_model, fr3_data)
                log_step += 1

                # CSV experiment logging is temporarily disabled.
                if ENABLE_CSV_LOGGING and log_step % 5 == 0:
                    log_path = ROOT / "logs" / "MAX_DQ_0.004_v2.csv"
                    log_path.parent.mkdir(parents=True, exist_ok=True)
                    log_exists = log_path.exists()
                    with log_path.open("a", newline="") as log_file:
                        fields = [
                            "timestamp",
                            "omy_position_x", "omy_position_y", "omy_position_z",
                            "fr3_target_position_x", "fr3_target_position_y", "fr3_target_position_z",
                            "fr3_actual_position_x", "fr3_actual_position_y", "fr3_actual_position_z",
                            "position_error_norm",
                            "omy_relative_angle_deg", "fr3_target_angle_deg",
                            "fr3_actual_angle_deg", "orientation_error_deg",
                            *[f"q{i}" for i in range(1, 8)],
                            *[f"q_cmd{i}" for i in range(1, 8)],
                            "max_joint_step",
                            "raw_max_dq", "cmd_max_dq", "dq_saturated", "dt",
                        ]
                        writer = csv.DictWriter(log_file, fieldnames=fields)
                        if not log_exists:
                            writer.writeheader()
                        row = {
                            "timestamp": time.monotonic(),
                            "position_error_norm": np.linalg.norm(position_error),
                            "omy_relative_angle_deg": rotation_matrix_to_angle_deg(omy_delta_rotation),
                            "fr3_target_angle_deg": rotation_matrix_to_angle_deg(
                                fr3_initial_rotation.T @ fr3_target_rotation
                            ),
                            "fr3_actual_angle_deg": rotation_matrix_to_angle_deg(
                                fr3_initial_rotation.T @ fr3_current_rotation
                            ),
                            "orientation_error_deg": rotation_matrix_to_angle_deg(
                                rotation_error_matrix
                            ),
                            "max_joint_step": max_joint_step,
                            "raw_max_dq": raw_max_dq,
                            "cmd_max_dq": cmd_max_dq,
                            "dq_saturated": dq_saturated,
                            "dt": dt,
                        }
                        row.update({
                            f"omy_position_{axis}": omy_current_position[index]
                            for index, axis in enumerate(("x", "y", "z"))
                        })
                        row.update({
                            f"fr3_target_position_{axis}": fr3_target_position[index]
                            for index, axis in enumerate(("x", "y", "z"))
                        })
                        row.update({
                            f"fr3_actual_position_{axis}": fr3_current_position[index]
                            for index, axis in enumerate(("x", "y", "z"))
                        })
                        row.update({f"q{i}": value for i, value in enumerate(q_current, 1)})
                        row.update({f"q_cmd{i}": value for i, value in enumerate(fr3_q_command, 1)})

                        writer.writerow(row)
                now = time.monotonic()
                if now - last_print_time >= 0.2:
                    tracking_error = np.linalg.norm(position_error)
                    rotational_tracking_error = np.linalg.norm(rotation_error)
                    return_error = np.linalg.norm(
                        fr3_current_position - fr3_initial_position
                    )
                    print("OMY current position:", omy_current_position)
                    print("OMY delta position:", delta_position_omy)
                    rotation_vector = rotation_matrix_to_rotvec(
                        omy_delta_rotation
                    )
                    rpy_degrees = np.degrees(
                        rotation_matrix_to_rpy(omy_delta_rotation)
                    )
                    print("OMY rotation vector:", rotation_vector)
                    print("OMY rotation angle:", np.linalg.norm(rotation_vector))
                    print("OMY relative RPY [deg] (roll, pitch, yaw):", rpy_degrees)
                    print("FR3 mapped delta position:", delta_position_fr3)
                    print("Trigger position:", trigger_position)
                    print("Position mode active:", position_mode_active)
                    print("FR3 target position:", fr3_target_position)
                    print("FR3 current position:", fr3_current_position)
                    print("Position error:", position_error)
                    print("FR3 rotation error vector:", rotation_error)
                    print(
                        "FR3 rotation error angle:",
                        rotational_tracking_error,
                    )
                    print("FR3 q current:", q_current)
                    print("FR3 q command:", fr3_q_command)
                    print(f"tracking error: {tracking_error * 1000:.2f} mm")
                    print(f"return error: {return_error * 1000:.2f} mm")
                    print(f"max joint step: {max_joint_step:.6f} rad")
                    print_orientation_debug(
                        omy_initial_rotation,
                        omy_current_rotation,
                        fr3_initial_rotation,
                        fr3_target_rotation,
                        fr3_current_rotation,
                    )
                    last_print_time = now

                viewer.sync()
                time.sleep(0.002)
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
