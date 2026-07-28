#!/usr/bin/python3
"""OMY-L100 -> MuJoCo FR3 6D Cartesian teleoperation.

This version separates:
- OMY-to-FR3 target mapping
- target rate limiting
- 6D damped least-squares velocity IK
- MuJoCo stepping
- low-rate viewer/logging

It targets a 1 kHz simulation/control step, but Python is not hard real-time.
For a real FR3, use the robot real-time interface and its measured period.
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


# ---------------------------------------------------------------------------
# Paths and runtime options
# ---------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parents[1]
OMY_MODEL_PATH = ROOT / "robotis_mujoco_menagerie" / "robotis_omy" / "scene.xml"
FR3_MODEL_PATH = ROOT / "mujoco_menagerie" / "franka_fr3" / "scene.xml"

ENABLE_VIEWER = True
ENABLE_LOGGING = True

CONTROL_HZ = 1000.0
CONTROL_DT = 1.0 / CONTROL_HZ
VIEWER_HZ = 30.0
LOG_HZ = 100.0
PRINT_HZ = 1.0
ROS_TIMEOUT_S = 0.20


# ---------------------------------------------------------------------------
# Teleoperation parameters
# ---------------------------------------------------------------------------

OMY_ROS_JOINTS = [f"joint{i}" for i in range(1, 7)]
OMY_MUJOCO_JOINTS = [f"Joint{i}" for i in range(1, 7)]
TRIGGER_JOINT = "rh_r1_joint"

TRIGGER_ON_THRESHOLD = -0.90
TRIGGER_OFF_THRESHOLD = -0.70

AXIS_MAP = np.array([
    [0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
])

# Small-angle equivalent of the previous orientation mapping:
# AXIS_MAP plus [roll, pitch, yaw] signs [1, -1, -1].
ROTATION_VECTOR_MAP = AXIS_MAP @ np.diag([1.0, -1.0, -1.0])

# Conservative initial values.
POSITION_SCALE = 0.60
ORIENTATION_SCALE = 0.30

# Cartesian target rate limits.
MAX_TARGET_LINEAR_SPEED = 0.10       # m/s
MAX_TARGET_ANGULAR_SPEED = 2.0      # rad/s
MAX_TARGET_ANGULAR_ACCEL = 2.0      # rad/s^2

# Task-space feedback used to generate a desired Cartesian twist.
POSITION_KP = 8.0                    # 1/s
ORIENTATION_KP = 4.0                 # 1/s

# DLS velocity IK.
IK_DAMPING = 0.05
ROTATION_LENGTH_SCALE = 0.10         # m/rad task metric
MAX_JOINT_SPEED = 0.80               # rad/s, simulation initial value


# ---------------------------------------------------------------------------
# ROS latest-state buffer
# ---------------------------------------------------------------------------

class OmyPose(Node):
    """Store the latest valid OMY leader state."""

    def __init__(self, joint_positions, state_lock):
        super().__init__("fr3_omy_bridge")
        self.joint_positions = joint_positions
        self.state_lock = state_lock
        self.trigger_position = 0.0
        self.last_message_time = 0.0
        self.has_joint_state = False

        self.subscription = self.create_subscription(
            JointState,
            "/leader/joint_states",
            self.joint_state_callback,
            10,
        )

    def joint_state_callback(self, message):
        received = dict(zip(message.name, message.position))
        required = OMY_ROS_JOINTS + [TRIGGER_JOINT]

        if not all(name in received for name in required):
            return

        with self.state_lock:
            for index, name in enumerate(OMY_ROS_JOINTS):
                self.joint_positions[index] = received[name]

            self.trigger_position = received[TRIGGER_JOINT]
            self.last_message_time = time.perf_counter()
            self.has_joint_state = True


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def keyframe_id(model, name):
    result = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
    if result < 0:
        raise ValueError(f"keyframe not found: {name}")
    return result


def site_id(model, name):
    result = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    if result < 0:
        raise ValueError(f"site not found: {name}")
    return result


def read_site_pose(data, site):
    position = data.site_xpos[site].copy()
    rotation = data.site_xmat[site].reshape(3, 3).copy()
    return position, rotation


def skew(vector):
    x, y, z = vector
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ])


def rotvec_to_matrix(rotvec):
    """SO(3) exponential map."""
    angle = float(np.linalg.norm(rotvec))

    if angle < 1e-9:
        return np.eye(3) + skew(rotvec)

    axis = rotvec / angle
    axis_skew = skew(axis)
    return (
        np.eye(3)
        + np.sin(angle) * axis_skew
        + (1.0 - np.cos(angle)) * (axis_skew @ axis_skew)
    )


def matrix_to_rotvec(rotation):
    """SO(3) logarithm map."""
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = float(np.arccos(cosine))

    if angle < 1e-7:
        return 0.5 * np.array([
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ])

    if np.pi - angle < 1e-5:
        axis = np.sqrt(np.maximum(np.diag(rotation) + 1.0, 0.0) * 0.5)
        major = int(np.argmax(axis))
        if axis[major] < 1e-7:
            return np.zeros(3)

        for index in range(3):
            if index != major:
                axis[index] = (
                    rotation[major, index] + rotation[index, major]
                ) / (4.0 * axis[major])

        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-9:
            return np.zeros(3)
        return angle * axis / axis_norm

    vee = np.array([
        rotation[2, 1] - rotation[1, 2],
        rotation[0, 2] - rotation[2, 0],
        rotation[1, 0] - rotation[0, 1],
    ])
    return angle * vee / (2.0 * np.sin(angle))


def limit_norm(vector, max_norm):
    norm = float(np.linalg.norm(vector))
    if norm <= max_norm or norm < 1e-12:
        return vector
    return vector * (max_norm / norm)


# ---------------------------------------------------------------------------
# Target generation and conditioning
# ---------------------------------------------------------------------------

def make_desired_target(
    omy_anchor_position,
    omy_anchor_rotation,
    omy_current_position,
    omy_current_rotation,
    fr3_anchor_position,
    fr3_anchor_rotation,
):
    """Map the OMY clutch-relative pose to an unconstrained FR3 target."""
    delta_position_omy = omy_current_position - omy_anchor_position
    desired_position = (
        fr3_anchor_position
        + POSITION_SCALE * (AXIS_MAP @ delta_position_omy)
    )

    omy_relative_rotation = omy_anchor_rotation.T @ omy_current_rotation
    omy_relative_rotvec = matrix_to_rotvec(omy_relative_rotation)
    mapped_rotvec = (
        ORIENTATION_SCALE
        * (ROTATION_VECTOR_MAP @ omy_relative_rotvec)
    )
    desired_rotation = fr3_anchor_rotation @ rotvec_to_matrix(mapped_rotvec)

    return desired_position, desired_rotation


def condition_target(
    current_target_position,
    current_target_rotation,
    desired_position,
    desired_rotation,
    previous_angular_velocity,
    dt,
):
    """Rate- and acceleration-limit the Cartesian target."""
    position_step = desired_position - current_target_position
    position_step = limit_norm(
        position_step,
        MAX_TARGET_LINEAR_SPEED * dt,
    )
    next_position = current_target_position + position_step

    target_rotation_error = desired_rotation @ current_target_rotation.T
    desired_angular_velocity = matrix_to_rotvec(target_rotation_error)
    desired_angular_velocity /= max(dt, 1e-9)
    desired_angular_velocity = limit_norm(
        desired_angular_velocity,
        MAX_TARGET_ANGULAR_SPEED,
    )
    delta_angular_velocity = desired_angular_velocity - previous_angular_velocity
    delta_angular_velocity = limit_norm(
        delta_angular_velocity,
        MAX_TARGET_ANGULAR_ACCEL * dt,
    )
    angular_velocity = previous_angular_velocity + delta_angular_velocity
    rotation_step = angular_velocity * dt
    next_rotation = (
        rotvec_to_matrix(rotation_step)
        @ current_target_rotation
    )

    linear_speed = float(np.linalg.norm(position_step) / max(dt, 1e-9))
    angular_speed = float(np.linalg.norm(angular_velocity))
    return next_position, next_rotation, linear_speed, angular_speed, angular_velocity


# ---------------------------------------------------------------------------
# 6D DLS velocity IK
# ---------------------------------------------------------------------------

def compute_joint_target(
    model,
    data,
    ee_site_id,
    dof_indices,
    qpos_indices,
    joint_lower,
    joint_upper,
    target_position,
    target_rotation,
    command_reference,
    jacp,
    jacr,
    dt,
):
    """Compute q_target = q_current + qdot*dt with 6D DLS."""
    current_position, current_rotation = read_site_pose(data, ee_site_id)

    position_error = target_position - current_position
    rotation_error_matrix = target_rotation @ current_rotation.T
    rotation_error = matrix_to_rotvec(rotation_error_matrix)

    desired_linear_velocity = limit_norm(
        POSITION_KP * position_error,
        MAX_TARGET_LINEAR_SPEED,
    )
    desired_angular_velocity = limit_norm(
        ORIENTATION_KP * rotation_error,
        MAX_TARGET_ANGULAR_SPEED,
    )

    jacp.fill(0.0)
    jacr.fill(0.0)
    mujoco.mj_jacSite(
        model,
        data,
        jacp,
        jacr,
        ee_site_id,
    )

    j_pos = jacp[:, dof_indices]
    j_rot = jacr[:, dof_indices]

    # ROTATION_LENGTH_SCALE converts angular velocity to a length metric.
    task_jacobian = np.vstack((
        j_pos,
        ROTATION_LENGTH_SCALE * j_rot,
    ))
    task_velocity = np.concatenate((
        desired_linear_velocity,
        ROTATION_LENGTH_SCALE * desired_angular_velocity,
    ))

    regularized = (
        task_jacobian @ task_jacobian.T
        + (IK_DAMPING ** 2) * np.eye(6)
    )
    qdot_raw = task_jacobian.T @ np.linalg.solve(
        regularized,
        task_velocity,
    )

    qdot_command = np.clip(
        qdot_raw,
        -MAX_JOINT_SPEED,
        MAX_JOINT_SPEED,
    )
    saturated = bool(np.max(np.abs(qdot_raw)) > MAX_JOINT_SPEED)

    q_current = data.qpos[qpos_indices].copy()

    # Use the held actuator command as the integration reference. Using the
    # sagging actual q here would make gravity-induced drift continue when
    # teleoperation becomes active.
    q_target = np.clip(
        command_reference + qdot_command * dt,
        joint_lower,
        joint_upper,
    )

    singular_values = np.linalg.svd(task_jacobian, compute_uv=False)
    condition_number = float(
        singular_values[0] / max(singular_values[-1], 1e-9)
    )

    diagnostics = {
        "position_error_m": float(np.linalg.norm(position_error)),
        "orientation_error_deg": float(
            np.degrees(np.linalg.norm(rotation_error))
        ),
        "raw_max_qdot": float(np.max(np.abs(qdot_raw))),
        "cmd_max_qdot": float(np.max(np.abs(qdot_command))),
        "qdot_saturated": saturated,
        "jacobian_condition": condition_number,
        "max_q_command_error": float(
            np.max(np.abs(q_target - q_current))
        ),
    }
    return q_target, diagnostics


# ---------------------------------------------------------------------------
# Viewer and logging
# ---------------------------------------------------------------------------

def draw_target_marker(viewer, position, rotation):
    if viewer is None:
        return

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

    colors = (
        np.array([1.0, 0.0, 0.0, 1.0]),
        np.array([0.0, 1.0, 0.0, 1.0]),
        np.array([0.0, 0.0, 1.0, 1.0]),
    )
    length = 0.045
    radius = 0.004

    for geom_index, axis_index in enumerate(range(3), start=1):
        axis = rotation[:, axis_index]
        other_a = rotation[:, (axis_index + 1) % 3]
        other_b = rotation[:, (axis_index + 2) % 3]
        geom_rotation = np.column_stack((other_a, other_b, axis))
        geom_position = position + 0.5 * length * axis

        mujoco.mjv_initGeom(
            scene.geoms[geom_index],
            mujoco.mjtGeom.mjGEOM_BOX,
            np.array([radius, radius, 0.5 * length]),
            geom_position,
            geom_rotation.reshape(-1),
            colors[axis_index],
        )


def write_log(log_path, rows):
    if not rows:
        return

    log_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())

    with log_path.open("w", newline="") as log_file:
        writer = csv.DictWriter(log_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rclpy.init()

    # OMY model: FK only.
    omy_model = mujoco.MjModel.from_xml_path(str(OMY_MODEL_PATH))
    omy_data = mujoco.MjData(omy_model)
    mujoco.mj_resetDataKeyframe(
        omy_model,
        omy_data,
        keyframe_id(omy_model, "home"),
    )
    mujoco.mj_forward(omy_model, omy_data)
    omy_ee_site_id = site_id(omy_model, "omy_ee_site")

    # FR3 model: IK + actuator dynamics.
    fr3_model = mujoco.MjModel.from_xml_path(str(FR3_MODEL_PATH))
    fr3_model.opt.timestep = CONTROL_DT
    fr3_data = mujoco.MjData(fr3_model)
    mujoco.mj_resetDataKeyframe(
        fr3_model,
        fr3_data,
        keyframe_id(fr3_model, "home"),
    )
    mujoco.mj_forward(fr3_model, fr3_data)
    fr3_ee_site_id = site_id(fr3_model, "attachment_site")

    fr3_joint_ids = np.array(
        [fr3_model.joint(f"fr3_joint{i}").id for i in range(1, 8)],
        dtype=int,
    )
    fr3_qpos_indices = np.array(
        [fr3_model.jnt_qposadr[index] for index in fr3_joint_ids],
        dtype=int,
    )
    fr3_dof_indices = np.array(
        [fr3_model.jnt_dofadr[index] for index in fr3_joint_ids],
        dtype=int,
    )
    fr3_joint_lower = fr3_model.jnt_range[fr3_joint_ids, 0].copy()
    fr3_joint_upper = fr3_model.jnt_range[fr3_joint_ids, 1].copy()
    fr3_actuator_indices = np.array(
        [fr3_model.actuator(f"fr3_joint{i}").id for i in range(1, 8)],
        dtype=int,
    )

    q_home = fr3_data.qpos[fr3_qpos_indices].copy()
    fr3_data.ctrl[fr3_actuator_indices] = q_home
    mujoco.mj_forward(fr3_model, fr3_data)

    fr3_target_position, fr3_target_rotation = read_site_pose(
        fr3_data,
        fr3_ee_site_id,
    )
    hold_q_target = q_home.copy()

    jacp = np.zeros((3, fr3_model.nv))
    jacr = np.zeros((3, fr3_model.nv))

    omy_qpos_addresses = [
        omy_model.jnt_qposadr[omy_model.joint(name).id]
        for name in OMY_MUJOCO_JOINTS
    ]

    joint_positions = omy_data.qpos[omy_qpos_addresses].copy()
    state_lock = threading.Lock()
    ros_node = OmyPose(joint_positions, state_lock)
    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True,
    )
    ros_thread.start()

    teleop_active = False
    target_angular_velocity = np.zeros(3)
    omy_anchor_position = None
    omy_anchor_rotation = None
    fr3_anchor_position = fr3_target_position.copy()
    fr3_anchor_rotation = fr3_target_rotation.copy()

    next_tick = time.perf_counter()
    last_viewer_sync = next_tick
    last_print = next_tick
    last_rate_report = next_tick
    cycles_since_report = 0
    deadline_misses = 0
    rows = []

    run_id = time.strftime("%Y%m%d_%H%M%S")
    log_path = ROOT / "logs" / f"refactored_teleop_{run_id}.csv"

    viewer_context = (
        mujoco.viewer.launch_passive(fr3_model, fr3_data)
        if ENABLE_VIEWER
        else None
    )

    try:
        if viewer_context is None:
            viewer = None
            keep_running = lambda: rclpy.ok()
        else:
            viewer = viewer_context.__enter__()
            keep_running = lambda: viewer.is_running() and rclpy.ok()

        print(f"Configured control rate: {CONTROL_HZ:.0f} Hz")
        print(f"MuJoCo timestep: {fr3_model.opt.timestep:.6f} s")
        print("Waiting for /leader/joint_states...")

        step_index = 0

        while keep_running():
            cycle_start = time.perf_counter()

            with state_lock:
                omy_target = joint_positions.copy()
                trigger_position = ros_node.trigger_position
                last_message_time = ros_node.last_message_time
                has_joint_state = ros_node.has_joint_state

            ros_fresh = (
                has_joint_state
                and cycle_start - last_message_time <= ROS_TIMEOUT_S
            )

            if ros_fresh:
                for address, position in zip(omy_qpos_addresses, omy_target):
                    omy_data.qpos[address] = position
                mujoco.mj_forward(omy_model, omy_data)

                omy_current_position, omy_current_rotation = read_site_pose(
                    omy_data,
                    omy_ee_site_id,
                )

                if (
                    not teleop_active
                    and trigger_position <= TRIGGER_ON_THRESHOLD
                ):
                    omy_anchor_position = omy_current_position.copy()
                    omy_anchor_rotation = omy_current_rotation.copy()
                    fr3_anchor_position = fr3_target_position.copy()
                    fr3_anchor_rotation = fr3_target_rotation.copy()
                    teleop_active = True
                    print("Teleoperation ON: new target-based anchor pair")

                elif (
                    teleop_active
                    and trigger_position >= TRIGGER_OFF_THRESHOLD
                ):
                    teleop_active = False
                    print("Teleoperation OFF: holding last Cartesian target")

                if teleop_active:
                    desired_position, desired_rotation = make_desired_target(
                        omy_anchor_position,
                        omy_anchor_rotation,
                        omy_current_position,
                        omy_current_rotation,
                        fr3_anchor_position,
                        fr3_anchor_rotation,
                    )
                    (
                        fr3_target_position,
                        fr3_target_rotation,
                        target_linear_speed,
                        target_angular_speed,
                        target_angular_velocity,
                    ) = condition_target(
                        fr3_target_position,
                        fr3_target_rotation,
                        desired_position,
                        desired_rotation,
                        target_angular_velocity,
                        CONTROL_DT,
                    )
                else:
                    target_linear_speed = 0.0
                    target_angular_speed = 0.0
                    target_angular_velocity = np.zeros(3)
            else:
                teleop_active = False
                target_linear_speed = 0.0
                target_angular_speed = 0.0
                target_angular_velocity = np.zeros(3)

            if teleop_active:
                q_target, diagnostics = compute_joint_target(
                    fr3_model,
                    fr3_data,
                    fr3_ee_site_id,
                    fr3_dof_indices,
                    fr3_qpos_indices,
                    fr3_joint_lower,
                    fr3_joint_upper,
                    fr3_target_position,
                    fr3_target_rotation,
                    hold_q_target,
                    jacp,
                    jacr,
                    CONTROL_DT,
                )
                hold_q_target = q_target.copy()
            else:
                # Do not recompute the command from sagging q_current while
                # gravity acts. Hold the last position target until teleop
                # resumes; this keeps the simulated FR3 at its posture.
                q_target = hold_q_target.copy()
                diagnostics = {
                    "position_error_m": 0.0,
                    "orientation_error_deg": 0.0,
                    "raw_max_qdot": 0.0,
                    "cmd_max_qdot": 0.0,
                    "qdot_saturated": False,
                    "jacobian_condition": 0.0,
                    "max_q_command_error": float(
                        np.max(np.abs(q_target - fr3_data.qpos[fr3_qpos_indices]))
                    ),
                }

            fr3_data.ctrl[fr3_actuator_indices] = q_target
            mujoco.mj_step(fr3_model, fr3_data)

            now = time.perf_counter()
            step_index += 1
            cycles_since_report += 1

            if (
                ENABLE_VIEWER
                and viewer is not None
                and now - last_viewer_sync >= 1.0 / VIEWER_HZ
            ):
                draw_target_marker(
                    viewer,
                    fr3_target_position,
                    fr3_target_rotation,
                )
                viewer.sync()
                last_viewer_sync = now

            if (
                ENABLE_LOGGING
                and step_index % max(1, int(CONTROL_HZ / LOG_HZ)) == 0
            ):
                rows.append({
                    "wall_time": now,
                    "sim_time": float(fr3_data.time),
                    "teleop_active": int(teleop_active),
                    "ros_fresh": int(ros_fresh),
                    "position_error_mm": 1000.0 * diagnostics["position_error_m"],
                    "orientation_error_deg": diagnostics["orientation_error_deg"],
                    "target_linear_speed_mps": target_linear_speed,
                    "target_angular_speed_radps": target_angular_speed,
                    "raw_max_qdot_radps": diagnostics["raw_max_qdot"],
                    "cmd_max_qdot_radps": diagnostics["cmd_max_qdot"],
                    "qdot_saturated": int(diagnostics["qdot_saturated"]),
                    "jacobian_condition": diagnostics["jacobian_condition"],
                    "max_q_command_error_rad": diagnostics[
                        "max_q_command_error"
                    ],
                    "deadline_misses": deadline_misses,
                })

            if now - last_print >= 1.0 / PRINT_HZ:
                print(
                    f"pos={1000.0 * diagnostics['position_error_m']:.2f} mm | "
                    f"rot={diagnostics['orientation_error_deg']:.2f} deg | "
                    f"qdot={diagnostics['cmd_max_qdot']:.3f} rad/s | "
                    f"cond={diagnostics['jacobian_condition']:.1f}"
                )
                last_print = now

            if now - last_rate_report >= 1.0:
                measured_hz = cycles_since_report / (now - last_rate_report)
                print(
                    f"measured loop={measured_hz:.1f} Hz | "
                    f"deadline misses={deadline_misses}"
                )
                cycles_since_report = 0
                deadline_misses = 0
                last_rate_report = now

            next_tick += CONTROL_DT
            remaining = next_tick - time.perf_counter()

            if remaining > 0.0:
                time.sleep(remaining)
            else:
                deadline_misses += 1
                if remaining < -5.0 * CONTROL_DT:
                    next_tick = time.perf_counter()

    finally:
        if viewer_context is not None:
            viewer_context.__exit__(None, None, None)

        if ENABLE_LOGGING:
            write_log(log_path, rows)
            print(f"Log written: {log_path}")

        ros_node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()