// ============================================================================
//  minibot_hardware/stepper_diffdrive.cpp
// ============================================================================

#include "minibot_hardware/stepper_diffdrive.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kTwoPi = 6.283185307179586;

// Mirrors MB_FLAG_* in stm32/Core/Inc/minibot_protocol.h
constexpr uint16_t kFlagEnabled    = 1u << 0;
constexpr uint16_t kFlagEstop      = 1u << 1;
constexpr uint16_t kFlagCmdTimeout = 1u << 2;
constexpr uint16_t kFlagCrcErr     = 1u << 3;
constexpr uint16_t kFlagOverrun    = 1u << 4;
constexpr uint16_t kFlagClamped    = 1u << 5;

rclcpp::Logger logger() { return rclcpp::get_logger("StepperDiffDrive"); }
rclcpp::Clock & steady()
{
  static rclcpp::Clock c(RCL_STEADY_TIME);
  return c;
}
}  // namespace

namespace minibot_hardware
{

// ---------------------------------------------------------------------------
//  Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn StepperDiffDrive::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & p = info_.hardware_parameters;
  auto get = [&](const std::string & k, const std::string & dflt) {
    auto it = p.find(k);
    return (it != p.end()) ? it->second : dflt;
  };

  device_           = get("device", "/dev/ttyAMA0");
  baud_rate_        = std::stoi(get("baud_rate", "115200"));
  timeout_ms_       = std::stoi(get("timeout_ms", "1000"));
  left_wheel_name_  = get("left_wheel_name", "left_wheel_joint");
  right_wheel_name_ = get("right_wheel_name", "right_wheel_joint");
  steps_per_rev_    = std::stod(get("steps_per_rev", "3200"));
  max_step_rate_    = std::stoi(get("max_step_rate", "1066"));
  left_invert_      = (get("left_invert", "false") == "true");
  right_invert_     = (get("right_invert", "false") == "true");

  if (steps_per_rev_ <= 0.0) {
    RCLCPP_FATAL(logger(), "steps_per_rev must be > 0 (got %f)", steps_per_rev_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  rad_per_step_  = kTwoPi / steps_per_rev_;
  steps_per_rad_ = steps_per_rev_ / kTwoPi;

  // Contract check. Fail LOUDLY at init rather than producing garbage odometry
  // three layers up where nobody can trace it back to here.
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(logger(),
        "Joint '%s' must expose exactly ONE velocity command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(logger(),
        "Joint '%s' must expose position AND velocity state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(logger(),
    "STM32 stepper base | %s @ %d | %.0f steps/rev (%.6f rad/step) | cap %d sps",
    device_.c_str(), baud_rate_, steps_per_rev_, rad_per_step_, max_step_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> StepperDiffDrive::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> s;
  s.emplace_back(left_wheel_name_,  hardware_interface::HW_IF_POSITION, &left_.pos);
  s.emplace_back(left_wheel_name_,  hardware_interface::HW_IF_VELOCITY, &left_.vel);
  s.emplace_back(right_wheel_name_, hardware_interface::HW_IF_POSITION, &right_.pos);
  s.emplace_back(right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &right_.vel);
  return s;
}

std::vector<hardware_interface::CommandInterface>
StepperDiffDrive::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> c;
  c.emplace_back(left_wheel_name_,  hardware_interface::HW_IF_VELOCITY, &left_.cmd);
  c.emplace_back(right_wheel_name_, hardware_interface::HW_IF_VELOCITY, &right_.cmd);
  return c;
}

hardware_interface::CallbackReturn StepperDiffDrive::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!link_.open(device_, baud_rate_)) {
    RCLCPP_FATAL(logger(), "%s", link_.last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  link_.send_ping();
  RCLCPP_INFO(logger(), "Serial port %s open.", device_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperDiffDrive::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  link_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperDiffDrive::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!link_.is_open() && !link_.open(device_, baud_rate_)) {
    RCLCPP_FATAL(logger(), "%s", link_.last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  left_  = Wheel{};
  right_ = Wheel{};
  have_frame_ = false;
  seq_valid_  = false;
  dropped_frames_ = 0;
  crc_errors_ = 0;
  mcu_flags_ = 0;

  link_.send_velocity(0, 0);
  link_.send_enable(true);      // energise -> holding torque

  RCLCPP_INFO(logger(), "Stepper base ACTIVE.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperDiffDrive::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (link_.is_open()) {
    link_.send_velocity(0, 0);
    link_.send_estop();
    link_.send_enable(false);   // de-energise: a stationary stepper at full
                                // current is a 10 W heater doing no work
  }
  RCLCPP_INFO(logger(),
    "Stepper base DEACTIVATED. Link stats: %lu dropped frames, %lu CRC errors.",
    static_cast<unsigned long>(dropped_frames_),
    static_cast<unsigned long>(crc_errors_));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperDiffDrive::on_error(
  const rclcpp_lifecycle::State &)
{
  // Whatever went wrong upstream, the motors must not keep running.
  if (link_.is_open()) {
    link_.send_estop();
    link_.send_enable(false);
  }
  RCLCPP_ERROR(logger(), "Hardware entered ERROR state -> motors stopped.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
//  read / write
// ---------------------------------------------------------------------------

hardware_interface::return_type StepperDiffDrive::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_.is_open()) return hardware_interface::return_type::ERROR;

  Feedback f;
  if (link_.poll(f, crc_errors_)) {

    // Sequence gap => the Pi missed a frame. We do NOT silently interpolate
    // across it: position comes from a cumulative counter so it self-heals, but
    // we count the gap so diagnostics can surface a flaky cable.
    if (seq_valid_) {
      const uint16_t expect = static_cast<uint16_t>(last_seq_ + 1);
      if (f.seq != expect) {
        dropped_frames_ += static_cast<uint16_t>(f.seq - expect);
      }
    }
    last_seq_  = f.seq;
    seq_valid_ = true;
    mcu_flags_ = f.flags;

    int32_t ls = f.left_steps,  rs = f.right_steps;
    int32_t lv = f.left_sps,    rv = f.right_sps;
    if (left_invert_)  { ls = -ls; lv = -lv; }
    if (right_invert_) { rs = -rs; rv = -rv; }

    // Capture an offset on the first frame rather than sending a reset. The
    // reset would race with frames already in flight; an offset cannot.
    if (!left_.offset_valid) {
      left_.offset  = ls;
      right_.offset = rs;
      left_.offset_valid = right_.offset_valid = true;
    }

    left_.steps  = ls;
    right_.steps = rs;
    left_.pos  = static_cast<double>(ls - left_.offset)  * rad_per_step_;
    right_.pos = static_cast<double>(rs - right_.offset) * rad_per_step_;
    left_.vel  = static_cast<double>(lv) * rad_per_step_;
    right_.vel = static_cast<double>(rv) * rad_per_step_;

    have_frame_    = true;
    last_frame_ns_ = static_cast<uint64_t>(steady().now().nanoseconds());

    if (f.flags & kFlagCmdTimeout) {
      RCLCPP_WARN_THROTTLE(logger(), steady(), 5000,
        "STM32 reports COMMAND TIMEOUT -- it stopped the motors because the Pi "
        "went quiet. Is controller_manager still running?");
    }
    if (f.flags & kFlagEstop) {
      RCLCPP_WARN_THROTTLE(logger(), steady(), 5000, "STM32 is in EMERGENCY STOP.");
    }
    if (f.flags & kFlagClamped) {
      RCLCPP_WARN_THROTTLE(logger(), steady(), 10000,
        "STM32 CLAMPED a velocity command -- Nav2 is asking for more than the "
        "20 RPM cap allows. Your limits disagree somewhere. Run "
        "tools/generate_config.py and re-paste.");
    }
    if (f.flags & kFlagOverrun) {
      RCLCPP_WARN_THROTTLE(logger(), steady(), 10000,
        "STM32 saw a UART overrun. Check cable/grounding.");
    }
  }

  if (!have_frame_) {
    // Still booting. Report zeros rather than erroring -- controller_manager
    // would otherwise kill the controller on the very first tick.
    return hardware_interface::return_type::OK;
  }

  const uint64_t now = static_cast<uint64_t>(steady().now().nanoseconds());
  if ((now - last_frame_ns_) > static_cast<uint64_t>(timeout_ms_) * 1000000ULL) {
    RCLCPP_ERROR_THROTTLE(logger(), steady(), 2000,
      "No valid frame from the STM32 for >%d ms. Check the cable, the baud rate, "
      "TX/RX orientation, and that grounds are common.", timeout_ms_);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepperDiffDrive::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_.is_open()) return hardware_interface::return_type::ERROR;

  // NaN guard. A NaN here becomes a garbage int32 on the wire and the robot
  // does something unpredictable. Refuse it.
  if (!std::isfinite(left_.cmd) || !std::isfinite(right_.cmd)) {
    RCLCPP_ERROR_THROTTLE(logger(), steady(), 1000,
      "Non-finite wheel command (%f, %f) -> sending STOP.", left_.cmd, right_.cmd);
    link_.send_velocity(0, 0);
    return hardware_interface::return_type::OK;
  }

  double l = left_.cmd  * steps_per_rad_;    // rad/s -> steps/s
  double r = right_.cmd * steps_per_rad_;
  if (left_invert_)  l = -l;
  if (right_invert_) r = -r;

  // PROPORTIONAL saturation (per-wheel cap, curvature-preserving).
  //
  //   For a diff drive, wheel speed = v +/- w*(L/2). v_max and w_max are each
  //   derived assuming the OTHER axis is zero, so a command at BOTH maxima needs
  //   2*v_max on the outer wheel (0.1407 m/s -> 40 RPM, over the 0.08 m/s / 20
  //   RPM per-tyre limit). Clamping each wheel INDEPENDENTLY would cap that wheel
  //   but leave the inner one alone, changing v_left:v_right -> the robot drives
  //   a DIFFERENT arc than commanded.
  //
  //   Instead, if EITHER wheel is over the cap, scale BOTH by the same factor.
  //   The ratio v_left:v_right is preserved, so the robot follows the intended
  //   curvature, just slower. Guarantees no wheel is ever commanded above
  //   max_step_rate (= 20 RPM = 0.0704 m/s, < 0.08 m/s) from ANY combination of
  //   linear + angular velocity. The MCU clamp stays as the last-ditch net.
  const double cap = static_cast<double>(max_step_rate_);
  const double peak = std::max(std::fabs(l), std::fabs(r));
  if (peak > cap) {
    const double s = cap / peak;
    l *= s;
    r *= s;
  }

  if (!link_.send_velocity(static_cast<int32_t>(std::lround(l)),
                           static_cast<int32_t>(std::lround(r))))
  {
    RCLCPP_ERROR_THROTTLE(logger(), steady(), 1000, "%s", link_.last_error().c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace minibot_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  minibot_hardware::StepperDiffDrive, hardware_interface::SystemInterface)
