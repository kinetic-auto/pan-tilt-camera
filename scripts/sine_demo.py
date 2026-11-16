#!/usr/bin/env python3
"""
Sweep the camera along a sine wave: pan is the x axis, tilt is the y axis.

  ros2 run pan_tilt_camera sine_demo.py                       # defaults below
  ros2 run pan_tilt_camera sine_demo.py --width 160 --amplitude 25 --cycles 3 --passes 4
  ros2 run pan_tilt_camera sine_demo.py --dry-run             # print the path, don't move

The path:
  pan  = x, sweeping from center-pan - width/2 to center-pan + width/2 and back (one "pass" each way)
  tilt = center-tilt + amplitude * sin(2*pi * cycles * (x - x_start) / width)

Tilt is a function of the pan POSITION, not of time, so the traced shape is a true sine
wave in pan/tilt even though pan eases in and out at each end of a pass (cosine profile)
to turn around smoothly. Return passes retrace the same wave.

The whole path (approach + all passes) is sent as ONE trajectory; Ctrl+C cancels and the
camera holds position. The path is checked against the joint limits before anything moves.
"""
import argparse
import math

import rclpy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Shared node / limits / action helpers (installed next to this script)
from circle_demo import ACTION, APPROACH_SPEED_DEG_S, DT, JOINTS, NAMESPACE, CircleDemo, to_duration


def sine_pass(center_pan, center_tilt, width, amplitude, cycles, n_points, reverse):
    """(pan, tilt) in radians for one pass, n_points + 1 samples including both ends."""
    x0 = center_pan - width / 2
    path = []
    for i in range(n_points + 1):
        s = i / n_points
        u = (1 - math.cos(math.pi * s)) / 2  # 0 -> 1, zero velocity at both ends
        if reverse:
            u = 1 - u
        x = x0 + width * u
        y = center_tilt + amplitude * math.sin(2 * math.pi * cycles * u)
        path.append((x, y))
    return path


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--center-pan", type=float, default=0.0, help="deg (default 0)")
    ap.add_argument("--center-tilt", type=float, default=40.0, help="deg (default 40)")
    ap.add_argument("--width", type=float, default=120.0, help="total pan sweep, deg (default 120)")
    ap.add_argument("--amplitude", type=float, default=20.0, help="tilt amplitude, deg (default 20)")
    ap.add_argument("--cycles", type=float, default=2.0, help="sine cycles across the width (default 2)")
    ap.add_argument("--pass-time", type=float, default=12.0, help="seconds per pass across (default 12)")
    ap.add_argument("--passes", type=int, default=2, help="passes across; 2 = there and back (default 2)")
    ap.add_argument("--dry-run", action="store_true", help="print the path and limits check, don't move")
    args, _ = ap.parse_known_args()  # ignore ROS args

    if args.width <= 0 or args.amplitude < 0 or args.cycles <= 0:
        ap.error("--width and --cycles must be > 0, --amplitude >= 0")
    if args.pass_time <= 0 or args.passes < 1:
        ap.error("--pass-time must be > 0 and --passes >= 1")

    rclpy.init()
    node = CircleDemo("sine_demo")
    log = node.get_logger()
    try:
        n = max(16, int(round(args.pass_time / DT)))
        dt = args.pass_time / n
        rad = math.radians
        passes = [sine_pass(rad(args.center_pan), rad(args.center_tilt), rad(args.width),
                            rad(args.amplitude), args.cycles, n, reverse=(p % 2 == 1))
                  for p in range(args.passes)]
        # Join passes; drop each pass's first sample after the first (same as the previous end).
        path = passes[0] + [pt for p in passes[1:] for pt in p[1:]]

        pans = [math.degrees(p) for p, _ in path]
        tilts = [math.degrees(t) for _, t in path]
        log.info(f"Sine: pan {min(pans):.1f} to {max(pans):.1f} deg, tilt {min(tilts):.1f} to {max(tilts):.1f} deg, "
                 f"{args.cycles:g} cycles, {args.pass_time:.1f} s/pass x {args.passes}")

        # Refuse rather than clamp: clamping would flatten the wave.
        limits = node.read_limits()
        for i, j in enumerate(JOINTS):
            if j not in limits:
                continue
            lo, hi = (math.degrees(v) for v in limits[j])
            vals = pans if i == 0 else tilts
            if min(vals) < lo - 1e-6 or max(vals) > hi + 1e-6:
                log.error(f"{j} would sweep {min(vals):.1f} to {max(vals):.1f} deg, outside its limits "
                          f"{lo:.1f} to {hi:.1f}. Adjust the center, width or amplitude.")
                return 1

        # Velocities by central difference; zero at the very start and end.
        vels = [(0.0, 0.0)]
        for k in range(1, len(path) - 1):
            vels.append(tuple((b - a) / (2 * dt) for a, b in zip(path[k - 1], path[k + 1])))
        vels.append((0.0, 0.0))
        peak_pan = max(abs(v[0]) for v in vels)
        peak_tilt = max(abs(v[1]) for v in vels)
        log.info(f"  peak speed: pan {math.degrees(peak_pan):.0f} deg/s, tilt {math.degrees(peak_tilt):.0f} deg/s")

        if args.dry_run:
            for k in range(0, n + 1, max(1, n // 8)):
                log.info(f"  pan={pans[k]:7.2f}  tilt={tilts[k]:6.2f}")
            log.info("Dry run: nothing sent")
            return 0

        if not node.wait_for_state():
            log.error(f"No /{NAMESPACE}/joint_states. Is the launch running?")
            return 1
        if not node.client.wait_for_server(timeout_sec=5.0):
            log.error(f"Action server /{NAMESPACE}/{ACTION} not available. Is the controller running?")
            return 1

        start = node.current
        approach = max(0.5, math.degrees(max(abs(a - b) for a, b in zip(path[0], start))) / APPROACH_SPEED_DEG_S)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=list(pos), velocities=list(vel), time_from_start=to_duration(approach + k * dt))
            for k, (pos, vel) in enumerate(zip(path, vels))]

        total = approach + args.pass_time * args.passes
        log.info(f"Moving to the start point ({approach:.1f} s), then sweeping. Total {total:.1f} s. Ctrl+C to stop.")
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
            log.error(f"Sine sweep failed (error {result.error_code}): {result.error_string}")
            return 1
        log.info("Done")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
