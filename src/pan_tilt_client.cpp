#include "pan_tilt_camera/pan_tilt_client.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdarg>
#include <cstdio>

#include "urdf/model.h"

namespace pan_tilt_camera
{

using namespace std::chrono_literals;

namespace
{
std::atomic<bool> g_stop{false};
void on_signal(int) {g_stop = true;}
}  // namespace

std::string strf(const char * fmt, ...)
{
  va_list ap;
  va_start(ap, fmt);
  va_list ap2;
  va_copy(ap2, ap);
  const int n = std::vsnprintf(nullptr, 0, fmt, ap);
  va_end(ap);
  std::string out(n > 0 ? n : 0, '\0');
  std::vsnprintf(out.data(), out.size() + 1, fmt, ap2);
  va_end(ap2);
  return out;
}

builtin_interfaces::msg::Duration to_duration(double seconds)
{
  const auto ns = static_cast<int64_t>(std::llround(seconds * 1e9));
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(ns / 1000000000);
  d.nanosec = static_cast<uint32_t>(ns % 1000000000);
  return d;
}

std::vector<std::string> init(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  auto args = rclcpp::remove_ros_arguments(argc, argv);
  if (!args.empty()) {args.erase(args.begin());}
  return args;
}

bool stop_requested() {return g_stop.load();}

// ------------------------------------------------------------------------------- Args

Args::Args(std::vector<std::string> argv, std::string usage)
: usage_(std::move(usage))
{
  for (size_t i = 0; i < argv.size(); ++i) {
    const std::string & a = argv[i];
    if (a == "-h" || a == "--help") {
      help_ = true;
    } else if (a.rfind("--", 0) == 0) {
      const bool has_value = i + 1 < argv.size() && argv[i + 1].rfind("--", 0) != 0;
      opts_[a.substr(2)] = has_value ? std::optional<std::string>(argv[++i]) : std::nullopt;
    } else {
      positional_.push_back(a);
    }
  }
}

double Args::num(const std::string & key, double fallback)
{
  used_[key] = true;
  auto it = opts_.find(key);
  if (it == opts_.end()) {return fallback;}
  if (!it->second) {throw std::invalid_argument("--" + key + " needs a value");}
  try {
    size_t pos = 0;
    const double v = std::stod(*it->second, &pos);
    if (pos != it->second->size()) {throw std::invalid_argument("");}
    return v;
  } catch (const std::exception &) {
    throw std::invalid_argument("--" + key + ": '" + *it->second + "' is not a number");
  }
}

bool Args::flag(const std::string & key)
{
  used_[key] = true;
  auto it = opts_.find(key);
  if (it == opts_.end()) {return false;}
  if (it->second) {throw std::invalid_argument("--" + key + " takes no value");}
  return true;
}

std::optional<std::string> Args::leftovers() const
{
  for (const auto & [k, v] : opts_) {
    if (!used_.count(k)) {return "unknown option --" + k;}
  }
  return std::nullopt;
}

// ------------------------------------------------------------------------------- PanTiltClient

PanTiltClient::PanTiltClient(const std::string & name)
: rclcpp::Node(name, kNamespace)
{
  client_ = rclcpp_action::create_client<FJT>(this, kAction);
  sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [this](const sensor_msgs::msg::JointState & m) {on_joint_states(m);});
}

void PanTiltClient::on_joint_states(const sensor_msgs::msg::JointState & msg)
{
  PanTilt p{};
  for (size_t j = 0; j < kJoints.size(); ++j) {
    auto it = std::find(msg.name.begin(), msg.name.end(), kJoints[j]);
    if (it == msg.name.end()) {return;}
    p[j] = msg.position.at(static_cast<size_t>(it - msg.name.begin()));
  }
  current_ = p;
}

Limits PanTiltClient::read_limits(double timeout_s)
{
  Limits limits;
  auto params = std::make_shared<rclcpp::AsyncParametersClient>(this, "robot_state_publisher");
  if (!params->wait_for_service(std::chrono::duration<double>(timeout_s))) {
    RCLCPP_WARN(get_logger(), "robot_state_publisher not found; joint limits not checked");
    return limits;
  }
  auto fut = params->get_parameters({"robot_description"});
  if (rclcpp::spin_until_future_complete(get_node_base_interface(), fut,
    std::chrono::duration<double>(timeout_s)) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "Could not read robot_description; joint limits not checked");
    return limits;
  }
  urdf::Model model;
  if (!model.initString(fut.get().at(0).as_string())) {
    RCLCPP_WARN(get_logger(), "Could not parse robot_description; joint limits not checked");
    return limits;
  }
  for (const auto & j : kJoints) {
    auto joint = model.getJoint(j);
    if (joint && joint->limits) {limits[j] = {joint->limits->lower, joint->limits->upper};}
  }
  return limits;
}

bool PanTiltClient::wait_for_state(double timeout_s)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  while (!current_ && !stop_requested() && std::chrono::steady_clock::now() < end) {
    rclcpp::spin_some(get_node_base_interface());
    std::this_thread::sleep_for(20ms);
  }
  return current_.has_value();
}

std::optional<std::string> PanTiltClient::check_limits(
  const std::vector<PanTilt> & path, const Limits & limits) const
{
  for (size_t j = 0; j < kJoints.size(); ++j) {
    auto it = limits.find(kJoints[j]);
    if (it == limits.end() || path.empty()) {continue;}
    auto [mn, mx] = std::minmax_element(path.begin(), path.end(),
        [j](const PanTilt & a, const PanTilt & b) {return a[j] < b[j];});
    const double lo = it->second.first, hi = it->second.second;
    if ((*mn)[j] < lo - 1e-8 || (*mx)[j] > hi + 1e-8) {
      return strf("%s would sweep %.1f to %.1f deg, outside its limits %.1f to %.1f.",
               kJoints[j].c_str(), deg((*mn)[j]), deg((*mx)[j]), deg(lo), deg(hi));
    }
  }
  return std::nullopt;
}

double PanTiltClient::approach_time(const PanTilt & target) const
{
  const PanTilt start = current_.value_or(target);
  const double biggest = std::max(std::abs(target[0] - start[0]), std::abs(target[1] - start[1]));
  return std::max(0.5, deg(biggest) / kApproachSpeedDegS);
}

int PanTiltClient::execute(const trajectory_msgs::msg::JointTrajectory & traj, const std::string & what)
{
  const auto log = get_logger();
  const std::string action_name = strf("/%s/%s", kNamespace, kAction);
  if (!client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(log, "Action server %s not available. Is the controller running?", action_name.c_str());
    return 1;
  }
  FJT::Goal goal;
  goal.trajectory = traj;

  auto send = client_->async_send_goal(goal);
  while (rclcpp::spin_until_future_complete(get_node_base_interface(), send, 100ms) ==
    rclcpp::FutureReturnCode::TIMEOUT)
  {
    if (stop_requested()) {return 130;}
  }
  auto handle = send.get();
  if (!handle) {
    RCLCPP_ERROR(log, "Goal rejected");
    return 1;
  }

  auto result = client_->async_get_result(handle);
  while (rclcpp::spin_until_future_complete(get_node_base_interface(), result, 100ms) ==
    rclcpp::FutureReturnCode::TIMEOUT)
  {
    if (stop_requested()) {
      RCLCPP_WARN(log, "Stopping: cancelling goal, camera holds position");
      auto cancel = client_->async_cancel_goal(handle);
      rclcpp::spin_until_future_complete(get_node_base_interface(), cancel, 2s);
      return 130;
    }
  }
  const auto wrapped = result.get();
  if (!wrapped.result) {
    RCLCPP_ERROR(log, "%s failed: no result from the controller (code %d)", what.c_str(),
      static_cast<int>(wrapped.code));
    return 1;
  }
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED ||
    wrapped.result->error_code != FJT::Result::SUCCESSFUL)
  {
    RCLCPP_ERROR(log, "%s failed (error %d): %s", what.c_str(), wrapped.result->error_code,
      wrapped.result->error_string.c_str());
    return 1;
  }
  return 0;
}

}  // namespace pan_tilt_camera
