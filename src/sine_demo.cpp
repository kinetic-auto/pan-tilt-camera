// Sweep the camera along a sine wave: pan is the x axis, tilt is the y axis.
//
//   ros2 run pan_tilt_camera sine_demo                         # defaults below
//   ros2 run pan_tilt_camera sine_demo --width 160 --amplitude 25 --cycles 3 --passes 4
//   ros2 run pan_tilt_camera sine_demo --dry-run               # print the path, don't move
//
// The path:
//   pan  = x, sweeping from center-pan - width/2 to center-pan + width/2 and back (one "pass" each way)
//   tilt = center-tilt + amplitude * sin(2*pi * cycles * (x - x_start) / width)
//
// Tilt is a function of the pan POSITION, not of time, so the traced shape is a true sine wave
// in pan/tilt even though pan eases in and out at each end of a pass (cosine profile) to turn
// around smoothly. Return passes retrace the same wave.
//
// The whole path (approach + all passes) is sent as ONE trajectory; Ctrl+C cancels and the
// camera holds position. The path is checked against the joint limits before anything moves.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "pan_tilt_camera/pan_tilt_client.hpp"

using namespace pan_tilt_camera;

namespace
{
const char * kUsage =
  "usage: ros2 run pan_tilt_camera sine_demo [options]\n"
  "  --center-pan DEG   center of the pan sweep (default 0)\n"
  "  --center-tilt DEG  tilt the wave oscillates around (default 40)\n"
  "  --width DEG        total pan sweep (default 120)\n"
  "  --amplitude DEG    tilt amplitude (default 20)\n"
  "  --cycles N         sine cycles across the width (default 2)\n"
  "  --pass-time S      seconds per pass across (default 12)\n"
  "  --passes N         passes across; 2 = there and back (default 2)\n"
  "  --dry-run          print the path and limits check, don't move\n";

// One pass, n + 1 samples including both ends (radians).
std::vector<PanTilt> sine_pass(
  double center_pan, double center_tilt, double width, double amplitude,
  double cycles, int n, bool reverse)
{
  const double x0 = center_pan - width / 2;
  std::vector<PanTilt> path;
  for (int i = 0; i <= n; ++i) {
    const double s = static_cast<double>(i) / n;
    double u = (1 - std::cos(M_PI * s)) / 2;  // 0 -> 1, zero velocity at both ends
    if (reverse) {u = 1 - u;}
    path.push_back({x0 + width * u, center_tilt + amplitude * std::sin(2 * M_PI * cycles * u)});
  }
  return path;
}
}  // namespace

int main(int argc, char ** argv)
{
  Args args(init(argc, argv), kUsage);
  double center_pan, center_tilt, width, amplitude, cycles, pass_time;
  int passes;
  bool dry_run;
  try {
    center_pan = args.num("center-pan", 0.0);
    center_tilt = args.num("center-tilt", 40.0);
    width = args.num("width", 120.0);
    amplitude = args.num("amplitude", 20.0);
    cycles = args.num("cycles", 2.0);
    pass_time = args.num("pass-time", 12.0);
    passes = static_cast<int>(args.num("passes", 2));
    dry_run = args.flag("dry-run");
    if (auto err = args.leftovers()) {throw std::invalid_argument(*err);}
    if (!args.positional().empty()) {throw std::invalid_argument("unexpected argument " + args.positional()[0]);}
    if (width <= 0 || amplitude < 0 || cycles <= 0) {
      throw std::invalid_argument("--width and --cycles must be > 0, --amplitude >= 0");
    }
    if (pass_time <= 0 || passes < 1) {throw std::invalid_argument("--pass-time must be > 0 and --passes >= 1");}
  } catch (const std::invalid_argument & e) {
    std::fprintf(stderr, "%serror: %s\n", kUsage, e.what());
    rclcpp::shutdown();
    return 2;
  }
  if (args.help()) {
    std::printf("%s", kUsage);
    rclcpp::shutdown();
    return 0;
  }

  auto node = std::make_shared<PanTiltClient>("sine_demo");
  const auto log = node->get_logger();
  int rc = 0;
  do {
    const int n = std::max(16, static_cast<int>(std::lround(pass_time / kDt)));
    const double dt = pass_time / n;
    std::vector<PanTilt> path;
    for (int p = 0; p < passes; ++p) {
      auto one = sine_pass(rad(center_pan), rad(center_tilt), rad(width), rad(amplitude), cycles, n, p % 2 == 1);
      // Drop each later pass's first sample: it is the previous pass's last one.
      path.insert(path.end(), one.begin() + (p == 0 ? 0 : 1), one.end());
    }

    auto [pmin, pmax] = std::minmax_element(path.begin(), path.end(),
        [](auto & a, auto & b) {return a[0] < b[0];});
    auto [tmin, tmax] = std::minmax_element(path.begin(), path.end(),
        [](auto & a, auto & b) {return a[1] < b[1];});
    RCLCPP_INFO(log, "Sine: pan %.1f to %.1f deg, tilt %.1f to %.1f deg, %g cycles, %.1f s/pass x %d",
      deg((*pmin)[0]), deg((*pmax)[0]), deg((*tmin)[1]), deg((*tmax)[1]), cycles, pass_time, passes);

    // Refuse rather than clamp: clamping would flatten the wave.
    if (auto err = node->check_limits(path, node->read_limits())) {
      RCLCPP_ERROR(log, "%s Adjust the center, width or amplitude.", err->c_str());
      rc = 1;
      break;
    }

    // Velocities by central difference; zero at the very start and end.
    std::vector<PanTilt> vels(path.size(), PanTilt{0.0, 0.0});
    double peak_pan = 0, peak_tilt = 0;
    for (size_t k = 1; k + 1 < path.size(); ++k) {
      vels[k] = {(path[k + 1][0] - path[k - 1][0]) / (2 * dt), (path[k + 1][1] - path[k - 1][1]) / (2 * dt)};
      peak_pan = std::max(peak_pan, std::abs(vels[k][0]));
      peak_tilt = std::max(peak_tilt, std::abs(vels[k][1]));
    }
    RCLCPP_INFO(log, "  peak speed: pan %.0f deg/s, tilt %.0f deg/s", deg(peak_pan), deg(peak_tilt));

    if (dry_run) {
      for (int k = 0; k <= n; k += std::max(1, n / 8)) {
        RCLCPP_INFO(log, "  pan=%7.2f  tilt=%6.2f", deg(path[k][0]), deg(path[k][1]));
      }
      RCLCPP_INFO(log, "Dry run: nothing sent");
      break;
    }

    if (!node->wait_for_state()) {
      RCLCPP_ERROR(log, "No /%s/joint_states. Is the launch running?", kNamespace);
      rc = 1;
      break;
    }

    const double approach = node->approach_time(path[0]);
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = kJoints;
    for (size_t k = 0; k < path.size(); ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions = {path[k][0], path[k][1]};
      pt.velocities = {vels[k][0], vels[k][1]};
      pt.time_from_start = to_duration(approach + k * dt);
      traj.points.push_back(pt);
    }

    RCLCPP_INFO(log, "Moving to the start point (%.1f s), then sweeping. Total %.1f s. Ctrl+C to stop.",
      approach, approach + pass_time * passes);
    rc = node->execute(traj, "Sine sweep");
    if (rc == 0) {RCLCPP_INFO(log, "Done");}
  } while (false);

  node.reset();
  rclcpp::shutdown();
  return rc;
}
