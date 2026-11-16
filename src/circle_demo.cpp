// Sweep the camera's aim around a circle.
//
//   ros2 run pan_tilt_camera circle_demo                       # defaults below
//   ros2 run pan_tilt_camera circle_demo --radius 25 --center-tilt 40 --period 6 --loops 3
//   ros2 run pan_tilt_camera circle_demo --dry-run             # print the path, don't move
//
// How the circle is built:
//   The center direction C comes from (center-pan, center-tilt). Two unit axes perpendicular
//   to C are built: X (horizontal, sideways) and Y (vertical, "up" in the camera's view).
//   The aim direction is swept as
//       d(theta) = cos(r) * C + sin(r) * (cos(theta) * X + sin(theta) * Y)
//   so the aim stays exactly r degrees from the center: a round circle in the image, even at
//   high tilt where simply adding sin/cos to pan and tilt would give a squashed ellipse.
//   Each d is converted back to pan = atan2(dy, dx), tilt = atan2(dz, hypot(dx, dy)).
//
// The whole path (approach + all loops) is sent as ONE trajectory, so the joint trajectory
// controller interpolates it smoothly. Ctrl+C cancels the goal and the camera holds position.
// The path is checked against the joint limits from robot_description before anything moves.
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
  "usage: ros2 run pan_tilt_camera circle_demo [options]\n"
  "  --center-pan DEG   circle center pan (default 0)\n"
  "  --center-tilt DEG  circle center tilt (default 35)\n"
  "  --radius DEG       angular radius of the circle (default 25)\n"
  "  --period S         seconds per loop (default 8)\n"
  "  --loops N          number of loops (default 2)\n"
  "  --clockwise        reverse the direction\n"
  "  --dry-run          print the path and limits check, don't move\n";

using Vec3 = std::array<double, 3>;

Vec3 unit(const Vec3 & v)
{
  const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  return {v[0] / n, v[1] / n, v[2] / n};
}

Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// Unit aim vector for pan/tilt in radians (x forward, y left, z up).
Vec3 direction(double pan, double tilt)
{
  return {std::cos(tilt) * std::cos(pan), std::cos(tilt) * std::sin(pan), std::sin(tilt)};
}

// n samples of one full circle, starting at the top.
std::vector<PanTilt> circle_path(double center_pan, double center_tilt, double radius, int n, bool clockwise)
{
  const Vec3 c = direction(center_pan, center_tilt);
  const Vec3 x_axis = unit(cross({0.0, 0.0, 1.0}, c));  // horizontal axis perpendicular to the aim
  const Vec3 y_axis = cross(c, x_axis);                  // vertical axis perpendicular to the aim
  const double sign = clockwise ? -1.0 : 1.0;
  std::vector<PanTilt> path;
  for (int i = 0; i < n; ++i) {
    const double theta = M_PI / 2 + sign * 2 * M_PI * i / n;
    Vec3 d;
    for (int k = 0; k < 3; ++k) {
      d[k] = std::cos(radius) * c[k] +
        std::sin(radius) * (std::cos(theta) * x_axis[k] + std::sin(theta) * y_axis[k]);
    }
    path.push_back({std::atan2(d[1], d[0]), std::atan2(d[2], std::hypot(d[0], d[1]))});
  }
  return path;
}
}  // namespace

int main(int argc, char ** argv)
{
  Args args(init(argc, argv), kUsage);
  double center_pan, center_tilt, radius, period;
  int loops;
  bool clockwise, dry_run;
  try {
    center_pan = args.num("center-pan", 0.0);
    center_tilt = args.num("center-tilt", 35.0);
    radius = args.num("radius", 25.0);
    period = args.num("period", 8.0);
    loops = static_cast<int>(args.num("loops", 2));
    clockwise = args.flag("clockwise");
    dry_run = args.flag("dry-run");
    if (auto err = args.leftovers()) {throw std::invalid_argument(*err);}
    if (!args.positional().empty()) {throw std::invalid_argument("unexpected argument " + args.positional()[0]);}
    if (!(radius > 0 && radius < 90)) {throw std::invalid_argument("--radius must be between 0 and 90 deg");}
    if (period <= 0 || loops < 1) {throw std::invalid_argument("--period must be > 0 and --loops >= 1");}
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

  auto node = std::make_shared<PanTiltClient>("circle_demo");
  const auto log = node->get_logger();
  int rc = 0;
  do {
    const int n = std::max(16, static_cast<int>(std::lround(period / kDt)));
    const double dt = period / n;
    const auto path = circle_path(rad(center_pan), rad(center_tilt), rad(radius), n, clockwise);

    auto [pmin, pmax] = std::minmax_element(path.begin(), path.end(),
        [](auto & a, auto & b) {return a[0] < b[0];});
    auto [tmin, tmax] = std::minmax_element(path.begin(), path.end(),
        [](auto & a, auto & b) {return a[1] < b[1];});
    RCLCPP_INFO(log, "Circle: center pan=%.1f tilt=%.1f deg, radius %.1f deg, %.1f s/loop x %d",
      center_pan, center_tilt, radius, period, loops);
    RCLCPP_INFO(log, "  pan sweeps %.1f to %.1f deg, tilt sweeps %.1f to %.1f deg",
      deg((*pmin)[0]), deg((*pmax)[0]), deg((*tmin)[1]), deg((*tmax)[1]));

    // Refuse rather than clamp: clamping would flatten the circle.
    if (auto err = node->check_limits(path, node->read_limits())) {
      RCLCPP_ERROR(log, "%s Move the center or shrink the radius.", err->c_str());
      rc = 1;
      break;
    }

    double peak = 0;
    for (int k = 0; k < n; ++k) {
      const auto & a = path[k];
      const auto & b = path[(k + 1) % n];
      peak = std::max(peak, std::hypot(b[0] - a[0], b[1] - a[1]) / dt);
    }
    RCLCPP_INFO(log, "  peak joint speed %.0f deg/s", deg(peak));

    if (dry_run) {
      for (int k = 0; k < n; k += std::max(1, n / 8)) {
        RCLCPP_INFO(log, "  theta=%5.1f  pan=%7.2f  tilt=%6.2f", 360.0 * k / n, deg(path[k][0]), deg(path[k][1]));
      }
      RCLCPP_INFO(log, "Dry run: nothing sent");
      break;
    }

    if (!node->wait_for_state()) {
      RCLCPP_ERROR(log, "No /%s/joint_states. Is the launch running?", kNamespace);
      rc = 1;
      break;
    }

    // Approach the top of the circle, then go around `loops` times and end back at the top.
    const double approach = node->approach_time(path[0]);
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = kJoints;
    const int last = loops * n;
    for (int k = 0; k <= last; ++k) {
      trajectory_msgs::msg::JointTrajectoryPoint pt;
      const auto & pos = path[k % n];
      pt.positions = {pos[0], pos[1]};
      if (k == 0 || k == last) {
        pt.velocities = {0.0, 0.0};  // start and stop the circle at rest
      } else {
        const auto & prev = path[(k - 1 + n) % n];
        const auto & next = path[(k + 1) % n];
        pt.velocities = {(next[0] - prev[0]) / (2 * dt), (next[1] - prev[1]) / (2 * dt)};
      }
      pt.time_from_start = to_duration(approach + k * dt);
      traj.points.push_back(pt);
    }

    RCLCPP_INFO(log, "Moving to the start point (%.1f s), then circling. Total %.1f s. Ctrl+C to stop.",
      approach, approach + period * loops);
    rc = node->execute(traj, "Circle");
    if (rc == 0) {RCLCPP_INFO(log, "Done");}
  } while (false);

  node.reset();
  rclcpp::shutdown();
  return rc;
}
