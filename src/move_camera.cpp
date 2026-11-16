// Move the pan/tilt camera through the trajectory controller.
//
//   ros2 run pan_tilt_camera move_camera 45 20      # pan=45 deg, tilt=20 deg
//   ros2 run pan_tilt_camera move_camera            # demo sequence
//
// Both joints start together and arrive together (the joint trajectory controller interpolates
// them over the same duration). The duration is set by the larger move at kApproachSpeedDegS,
// and each move waits for the controller to report success before the next one starts.
// Targets are clamped to the joint limits read from robot_description, so the limits only
// live in pan_tilt_limits.xacro.
#include <cstdio>
#include <string>
#include <vector>

#include "pan_tilt_camera/pan_tilt_client.hpp"

using namespace pan_tilt_camera;

namespace
{
const char * kUsage =
  "usage: ros2 run pan_tilt_camera move_camera [PAN_DEG TILT_DEG]\n"
  "  With no arguments, runs a demo sequence.\n";

// Returns 0 on success, otherwise a process exit code.
int move(PanTiltClient & node, const Limits & limits, double pan_deg, double tilt_deg)
{
  const auto log = node.get_logger();
  if (!node.wait_for_state()) {
    RCLCPP_ERROR(log, "No /%s/joint_states. Is the launch running?", kNamespace);
    return 1;
  }
  PanTilt target{rad(pan_deg), rad(tilt_deg)};
  for (size_t j = 0; j < kJoints.size(); ++j) {
    auto it = limits.find(kJoints[j]);
    if (it == limits.end()) {continue;}
    const auto [lo, hi] = it->second;
    if (target[j] < lo || target[j] > hi) {
      RCLCPP_WARN(log, "%s %.1f deg is outside limits, clamping", kJoints[j].c_str(), deg(target[j]));
      target[j] = std::clamp(target[j], lo, hi);
    }
  }
  const double secs = node.approach_time(target);

  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = kJoints;
  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = {target[0], target[1]};
  pt.velocities = {0.0, 0.0};
  pt.time_from_start = to_duration(secs);
  traj.points.push_back(pt);

  RCLCPP_INFO(log, "Moving to pan=%.1f tilt=%.1f deg over %.1f s", deg(target[0]), deg(target[1]), secs);
  const int rc = node.execute(traj, "Move");
  if (rc == 0) {RCLCPP_INFO(log, "Arrived");}
  return rc;
}
}  // namespace

int main(int argc, char ** argv)
{
  Args args(init(argc, argv), kUsage);
  if (args.help()) {
    std::printf("%s", kUsage);
    rclcpp::shutdown();
    return 0;
  }
  if (auto err = args.leftovers(); err || (args.positional().size() != 0 && args.positional().size() != 2)) {
    std::fprintf(stderr, "%s%s\n", kUsage, err ? err->c_str() : "need both PAN_DEG and TILT_DEG");
    rclcpp::shutdown();
    return 2;
  }
  std::vector<std::pair<double, double>> moves;
  try {
    if (args.positional().size() == 2) {
      moves.emplace_back(std::stod(args.positional()[0]), std::stod(args.positional()[1]));
    } else {
      moves = {{0, 0}, {90, 30}, {-90, -30}, {45, 60}, {0, 0}};
    }
  } catch (const std::exception &) {
    std::fprintf(stderr, "%sPAN_DEG and TILT_DEG must be numbers\n", kUsage);
    rclcpp::shutdown();
    return 2;
  }

  auto node = std::make_shared<PanTiltClient>("move_camera");
  const Limits limits = node->read_limits();
  for (const auto & [j, lim] : limits) {
    RCLCPP_INFO(node->get_logger(), "%s limits: %.1f to %.1f deg", j.c_str(), deg(lim.first), deg(lim.second));
  }
  int rc = 0;
  for (const auto & [p, t] : moves) {
    if ((rc = move(*node, limits, p, t)) != 0) {break;}
  }
  node.reset();
  rclcpp::shutdown();
  return rc;
}
