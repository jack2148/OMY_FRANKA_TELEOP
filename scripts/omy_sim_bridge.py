#!/usr/bin/env python3

import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


MODEL_PATH = (
    Path("/home/chan/omy_franka_teleop")
    / "robotis_mujoco_menagerie"
    / "robotis_omy"
    / "scene.xml"
)

ROS_JOINTS = [
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "rh_r1_joint",
]


MUJOCO_ACTUATORS = [
    "Joint1",
    "Joint2",
    "Joint3",
    "Joint4",
    "Joint5",
    "Joint6",
    "Gripper",
]



class OmySimBridge(Node):
    def __init__(self, target_positions):
        super().__init__("omy_sim_bridge")

        self.target_positions = target_positions
        self.target_lock = threading.Lock()

        self.subscription = self.create_subscription(
            JointState,
            "/leader/joint_states",
            self.joint_state_callback,
            10,
        )
        
    def joint_state_callback(self, msg):
        received = dict(zip(msg.name, msg.position))

        with self.target_lock:
            for i, joint_name in enumerate(ROS_JOINTS):
                if joint_name in received:
                    self.target_positions[i] = received[joint_name]



def main():
    rclpy.init()

    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    target_positions = [0.0] * len(MUJOCO_ACTUATORS)

    ros_node = OmySimBridge(target_positions)
    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True,
    )
    ros_thread.start()

    actuator_ids = [
        model.actuator(name).id
        for name in MUJOCO_ACTUATORS
    ]

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running() and rclpy.ok():
            with ros_node.target_lock:
                for i, actuator_id in enumerate(actuator_ids):
                    data.ctrl[actuator_id] = target_positions[i]

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.002)

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()