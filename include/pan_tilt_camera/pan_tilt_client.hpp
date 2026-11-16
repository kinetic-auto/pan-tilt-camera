// Shared helpers for the pan/tilt command-line nodes (move_camera, circle_demo, sine_demo).
#pragma once

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace pan_tilt_camera
{

// All names are relative and resolve inside this namespace
// (override with: --ros-args -r __ns:=/other_ns).
inline constexpr char kNamespace[] = "pan_tilt_camera";
inline constexpr char kAction[] = "pan_tilt_controller/follow_joint_trajectory";
inline const std::vector<std::string> kJoints{"pan_joint", "tilt_joint"};
inline constexpr double kApproachSpeedDegS = 40.0;  // deg/s for point-to-point moves
inline constexpr double kDt = 0.05;                 // s between trajectory points

using PanTilt = std::array<double, 2>;              // {pan, tilt}, radians
using Limits = std::map<std::string, std::pair<double, double>>;  // joint -> {lower, upper}, radians

inline double deg(double r) {return r * 180.0 / M_PI;}
inline double rad(double d) {return d * M_PI / 180.0;}

// printf-style formatting into a std::string.
std::string strf(const char * fmt, ...) __attribute__((format(printf, 1, 2)));

// Seconds -> Duration. Rounds once and then splits, so 1.9999999999 s becomes 2 s (not 1 s).
builtin_interfaces::msg::Duration to_duration(double seconds);

// rclcpp::init without rclcpp's own Ctrl+C handler, so a running goal can still be cancelled
// on Ctrl+C. Returns the non-ROS arguments (argv[0] removed).
std::vector<std::string> init(int argc, char ** argv);
bool stop_requested();

// Minimal "--key value" / "--flag" parser. Unknown options are an error.
class Args
{
public:
  Args(std::vector<std::string> argv, std::string usage);
  double num(const std::string & key, double fallback);
  bool flag(const std::string & key);
  const std::vector<std::string> & positional() const {return positional_;}
  bool help() const {return help_;}
  const std::string & usage() const {return usage_;}
  // Call after reading every option: returns an error message for anything left unread.
  std::optional<std::string> leftovers() const;

private:
  std::map<std::string, std::optional<std::string>> opts_;
  std::map<std::string, bool> used_;
  std::vector<std::string> positional_;
  std::string usage_;
  bool help_ = false;
};

class PanTiltClient : public rclcpp::Node
{
public:
  using FJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FJT>;

  explicit PanTiltClient(const std::string & name);

  // Joint limits from robot_state_publisher's robot_description. Empty if unavailable.
  Limits read_limits(double timeout_s = 5.0);
  // Wait for the first joint_states message. False on timeout or Ctrl+C.
  bool wait_for_state(double timeout_s = 5.0);
  std::optional<PanTilt> current() const {return current_;}

  // Check a planned path against the limits. Returns an error message if any point is outside.
  std::optional<std::string> check_limits(const std::vector<PanTilt> & path, const Limits & limits) const;

  // Send a trajectory and wait for it to finish. Ctrl+C cancels the goal (camera holds position).
  // Returns a process exit code: 0 success, 1 failure, 130 cancelled.
  int execute(const trajectory_msgs::msg::JointTrajectory & traj, const std::string & what);

  // Seconds for a point-to-point move from the current position, at kApproachSpeedDegS (min 0.5 s).
  double approach_time(const PanTilt & target) const;

private:
  void on_joint_states(const sensor_msgs::msg::JointState & msg);

  rclcpp_action::Client<FJT>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  std::optional<PanTilt> current_;
};

}  // namespace pan_tilt_camera
