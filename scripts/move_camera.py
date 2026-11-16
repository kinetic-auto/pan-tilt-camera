#!/usr/bin/env python3
"""
Move the pan/tilt camera through the trajectory controller.

  ros2 run pan_tilt_camera move_camera.py 45 20      # pan=45 deg, tilt=20 deg
  ros2 run pan_tilt_camera move_camera.py            # demo sequence

Both joints start together and arrive together (the joint trajectory controller
interpolates them over the same duration). The duration is set by the larger
move at SPEED_DEG_S, and each move waits for the controller to report success
before the next one starts.

Targets are clamped to the joint limits read from /robot_description, so the
limits only live in pan_tilt_limits.xacro.
"""
import math
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ["pan_joint", "tilt_joint"]
SPEED_DEG_S = 40.0
# Relative names, resolved inside NAMESPACE (override with: --ros-args -r __ns:=/other_ns)
NAMESPACE = "pan_tilt_camera"
ACTION = "pan_tilt_controller/follow_joint_trajectory"


class CameraMover(Node):
    def __init__(self):
        super().__init__("move_camera", namespace=NAMESPACE)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.current = None
        self.create_subscription(JointState, "joint_states", self._on_joint_states, 10)
        self.limits = self._read_limits()

    def _on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.current = [pos[j] for j in JOINTS]

    def _read_limits(self):
        """Get <limit lower/upper> for each joint from robot_state_publisher's robot_description."""
        cli = self.create_client(GetParameters, "robot_state_publisher/get_parameters")
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("robot_state_publisher not found; joint limits not applied")
            return {}
        fut = cli.call_async(GetParameters.Request(names=["robot_description"]))
        rclpy.spin_until_future_complete(self, fut)
        urdf = fut.result().values[0].string_value
        limits = {}
        for joint in ET.fromstring(urdf).iter("joint"):
            lim = joint.find("limit")
            if joint.get("name") in JOINTS and lim is not None:
                limits[joint.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
        for j, (lo, hi) in limits.items():
            self.get_logger().info(f"{j} limits: {math.degrees(lo):.1f} to {math.degrees(hi):.1f} deg")
        return limits

    def move(self, pan_deg, tilt_deg):
        while self.current is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        target = []
        for j, deg in zip(JOINTS, (pan_deg, tilt_deg)):
            rad = math.radians(deg)
            if j in self.limits:
                lo, hi = self.limits[j]
                if not lo <= rad <= hi:
                    self.get_logger().warn(f"{j} {deg:.1f} deg is outside limits, clamping")
                rad = min(max(rad, lo), hi)
            target.append(rad)

        biggest = max(abs(t - c) for t, c in zip(target, self.current))
        secs = max(0.5, math.degrees(biggest) / SPEED_DEG_S)

        point = JointTrajectoryPoint()
        point.positions = target
        point.velocities = [0.0] * len(JOINTS)
        point.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [point]

        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server /{NAMESPACE}/{ACTION} not available. Is the controller running?")
            return False

        self.get_logger().info(
            f"Moving to pan={math.degrees(target[0]):.1f} tilt={math.degrees(target[1]):.1f} deg over {secs:.1f} s")
        send = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f"Move failed (error {result.error_code}): {result.error_string}")
            return False
        self.get_logger().info("Arrived")
        return True


def main():
    rclpy.init()
    node = CameraMover()
    try:
        if len(sys.argv) >= 3:
            node.move(float(sys.argv[1]), float(sys.argv[2]))
        else:
            for p, t in [(0, 0), (90, 30), (-90, -30), (45, 60), (0, 0)]:
                if not node.move(p, t):
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
