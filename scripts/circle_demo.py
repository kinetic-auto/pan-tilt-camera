#!/usr/bin/env python3
"""
Sweep the camera's aim around a circle.

  ros2 run pan_tilt_camera circle_demo.py                    # defaults below
  ros2 run pan_tilt_camera circle_demo.py --radius 25 --center-tilt 40 --period 6 --loops 3
  ros2 run pan_tilt_camera circle_demo.py --dry-run          # print the path, don't move

How the circle is built:
  The center direction C comes from (center-pan, center-tilt). Two unit axes perpendicular
  to C are built: X (horizontal, sideways) and Y (vertical, "up" in the camera's view).
  The aim direction is swept as
      d(theta) = cos(r) * C + sin(r) * (cos(theta) * X + sin(theta) * Y)
  so the aim stays exactly r degrees from the center: a round circle in the image, even at
  high tilt where simply adding sin/cos to pan and tilt would give a squashed ellipse.
  Each d is converted back to pan = atan2(dy, dx), tilt = atan2(dz, hypot(dx, dy)).

The whole path (approach + all loops) is sent as ONE trajectory, so the joint trajectory
controller interpolates it smoothly. Ctrl+C cancels the goal and the camera holds position.
The path is checked against the joint limits from /robot_description before anything moves.
"""
import argparse
import math
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
# Relative names, resolved inside NAMESPACE (override with: --ros-args -r __ns:=/other_ns)
NAMESPACE = "pan_tilt_camera"
ACTION = "pan_tilt_controller/follow_joint_trajectory"
APPROACH_SPEED_DEG_S = 40.0  # same as move_camera.py
DT = 0.05                    # s between trajectory points


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def direction(pan, tilt):
    """Unit aim vector for pan/tilt in radians (x forward, y left, z up)."""
    return [math.cos(tilt) * math.cos(pan), math.cos(tilt) * math.sin(pan), math.sin(tilt)]


def circle_path(center_pan, center_tilt, radius, n_points, clockwise):
    """List of (pan, tilt) in radians, n_points samples of one full circle, starting at the top."""
    c = direction(center_pan, center_tilt)
    x_axis = unit(cross([0.0, 0.0, 1.0], c))  # horizontal axis perpendicular to the aim
    y_axis = cross(c, x_axis)                 # vertical axis perpendicular to the aim
    sign = -1.0 if clockwise else 1.0
    path = []
    for i in range(n_points):
        theta = math.pi / 2 + sign * 2 * math.pi * i / n_points
        d = [math.cos(radius) * c[k] +
             math.sin(radius) * (math.cos(theta) * x_axis[k] + math.sin(theta) * y_axis[k])
             for k in range(3)]
        path.append((math.atan2(d[1], d[0]), math.atan2(d[2], math.hypot(d[0], d[1]))))
    return path


def to_duration(t):
    ns = int(round(t * 1e9))  # round once, then split, so 1.9999999999 s -> 2 s (not 1 s)
    return Duration(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class CircleDemo(Node):
    def __init__(self, name="circle_demo"):
        super().__init__(name, namespace=NAMESPACE)
        self.client = ActionClient(self, FollowJointTrajectory, ACTION)
        self.current = None
        self.create_subscription(JointState, "joint_states", self._on_joint_states, 10)

    def _on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.current = [pos[j] for j in JOINTS]

    def read_limits(self):
        cli = self.create_client(GetParameters, "robot_state_publisher/get_parameters")
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("robot_state_publisher not found; joint limits not checked")
            return {}
        fut = cli.call_async(GetParameters.Request(names=["robot_description"]))
        rclpy.spin_until_future_complete(self, fut)
        limits = {}
        for joint in ET.fromstring(fut.result().values[0].string_value).iter("joint"):
            lim = joint.find("limit")
            if joint.get("name") in JOINTS and lim is not None:
                limits[joint.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
        return limits

    def wait_for_state(self, timeout=5.0):
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while self.current is None and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current is not None


def build_trajectory(start, path, period, loops):
    """Approach from start to path[0], then go around `loops` times and end back at path[0]."""
    n = len(path)
    dt = period / n
    approach = max(0.5, math.degrees(max(abs(a - b) for a, b in zip(path[0], start))) / APPROACH_SPEED_DEG_S)
    # positions at each time step (the circle closes on itself, so index wraps)
    samples = [(approach + k * dt, path[k % n]) for k in range(loops * n + 1)]
    points = []
    for k, (t, pos) in enumerate(samples):
        if k == 0 or k == len(samples) - 1:
            vel = (0.0, 0.0)  # start and stop the circle at rest
        else:
            prev, nxt = path[(k - 1) % n], path[(k + 1) % n]
            vel = tuple((b - a) / (2 * dt) for a, b in zip(prev, nxt))
        points.append(JointTrajectoryPoint(positions=list(pos), velocities=list(vel),
                                           time_from_start=to_duration(t)))
    return points, approach


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--center-pan", type=float, default=0.0, help="deg (default 0)")
    ap.add_argument("--center-tilt", type=float, default=35.0, help="deg (default 35)")
    ap.add_argument("--radius", type=float, default=25.0, help="angular radius of the circle, deg (default 25)")
    ap.add_argument("--period", type=float, default=8.0, help="seconds per loop (default 8)")
    ap.add_argument("--loops", type=int, default=2, help="number of loops (default 2)")
    ap.add_argument("--clockwise", action="store_true", help="reverse the direction")
    ap.add_argument("--dry-run", action="store_true", help="print the path and limits check, don't move")
    args, _ = ap.parse_known_args()  # ignore ROS args

    if not 0 < args.radius < 90:
        ap.error("--radius must be between 0 and 90 deg")
    if args.period <= 0 or args.loops < 1:
        ap.error("--period must be > 0 and --loops >= 1")

    rclpy.init()
    node = CircleDemo()
    log = node.get_logger()
    try:
        n_points = max(16, int(round(args.period / DT)))
        path = circle_path(math.radians(args.center_pan), math.radians(args.center_tilt),
                           math.radians(args.radius), n_points, args.clockwise)

        pans = [math.degrees(p) for p, _ in path]
        tilts = [math.degrees(t) for _, t in path]
        log.info(f"Circle: center pan={args.center_pan:.1f} tilt={args.center_tilt:.1f} deg, "
                 f"radius {args.radius:.1f} deg, {args.period:.1f} s/loop x {args.loops}")
        log.info(f"  pan sweeps {min(pans):.1f} to {max(pans):.1f} deg, "
                 f"tilt sweeps {min(tilts):.1f} to {max(tilts):.1f} deg")

        # Refuse rather than clamp: clamping would flatten the circle.
        limits = node.read_limits()
        for i, j in enumerate(JOINTS):
            if j not in limits:
                continue
            lo, hi = (math.degrees(v) for v in limits[j])
            vals = pans if i == 0 else tilts
            if min(vals) < lo - 1e-6 or max(vals) > hi + 1e-6:
                log.error(f"{j} would sweep {min(vals):.1f} to {max(vals):.1f} deg, outside its limits "
                          f"{lo:.1f} to {hi:.1f}. Move the center or shrink the radius.")
                return 1

        peak = max(math.hypot(*(b - a for a, b in zip(path[k], path[(k + 1) % n_points])))
                   for k in range(n_points)) / (args.period / n_points)
        log.info(f"  peak joint speed {math.degrees(peak):.0f} deg/s")

        if args.dry_run:
            for k in range(0, n_points, max(1, n_points // 8)):
                log.info(f"  theta={360 * k / n_points:5.1f}  pan={pans[k]:7.2f}  tilt={tilts[k]:6.2f}")
            log.info("Dry run: nothing sent")
            return 0

        if not node.wait_for_state():
            log.error(f"No /{NAMESPACE}/joint_states. Is the launch running?")
            return 1
        if not node.client.wait_for_server(timeout_sec=5.0):
            log.error(f"Action server /{NAMESPACE}/{ACTION} not available. Is the controller running?")
            return 1

        points, approach = build_trajectory(node.current, path, args.period, args.loops)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = points

        total = approach + args.period * args.loops
        log.info(f"Moving to the start point ({approach:.1f} s), then circling. Total {total:.1f} s. Ctrl+C to stop.")
        send = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send)
        handle = send.result()
        if not handle.accepted:
            log.error("Goal rejected")
            return 1
        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(node, result_future)
        except KeyboardInterrupt:
            log.warn("Stopping: cancelling goal, camera holds position")
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2.0)
            return 130
        result = result_future.result().result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            log.error(f"Circle failed (error {result.error_code}): {result.error_string}")
            return 1
        log.info("Done")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
