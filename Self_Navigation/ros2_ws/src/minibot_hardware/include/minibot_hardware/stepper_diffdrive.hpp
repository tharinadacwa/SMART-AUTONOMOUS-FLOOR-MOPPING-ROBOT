// ============================================================================
//  minibot_hardware/stepper_diffdrive.hpp
//
//  ros2_control SystemInterface for a differential-drive base whose wheels are
//  NEMA-17 steppers driven by DRV8825s from an STM32F407.
//
//  WHAT IT REPLACES
//      diffdrive_arduino/DiffDriveArduinoHardware, which the original project
//      used. That plugin's serial protocol is built entirely around QUADRATURE
//      ENCODERS (it sends 'e' to read counts and runs a PID loop on the MCU).
//      This robot has no encoders. There is no parameter you can set to make it
//      work -- the plugin had to go.
//
//  WHERE ODOMETRY COMES FROM WITHOUT ENCODERS
//      The STM32 generates every STEP pulse itself, so it knows exactly how many
//      it emitted and in which direction. It reports that tally at 50 Hz. We
//      convert it to joint position:
//
//          position_rad = steps * 2*pi / steps_per_rev
//
//      This is not an estimate. It is a count of pulses that physically left the
//      pin. It is exact right up until the motor SKIPS a step -- which is why
//      the firmware's jerk-limited profile and the 20 RPM cap exist, and why the
//      MPU6050 is fused in downstream to catch the heading error that a skipped
//      step would otherwise hide.
//
//  INTERFACES (identical to the encoder version, so nothing downstream changes)
//      command : velocity (rad/s)
//      state   : position (rad), velocity (rad/s)
// ============================================================================

#ifndef MINIBOT_HARDWARE__STEPPER_DIFFDRIVE_HPP_
#define MINIBOT_HARDWARE__STEPPER_DIFFDRIVE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "minibot_hardware/serial_link.hpp"

namespace minibot_hardware
{

class StepperDiffDrive : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(StepperDiffDrive)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  SerialLink link_;

  // ---- parameters (from ros2_control.xacro) ------------------------------
  std::string device_{"/dev/ttyAMA0"};  // Pi 5 GPIO UART (PA9/PA10 <-> GPIO14/15)
  int         baud_rate_{115200};
  int         timeout_ms_{1000};
  std::string left_wheel_name_{"left_wheel_joint"};
  std::string right_wheel_name_{"right_wheel_joint"};
  double      steps_per_rev_{3200.0};
  int         max_step_rate_{1066};      // must match MB_MAX_STEP_RATE
  bool        left_invert_{false};
  bool        right_invert_{false};

  double rad_per_step_{0.0};
  double steps_per_rad_{0.0};

  // ---- wheel state -------------------------------------------------------
  struct Wheel
  {
    int32_t steps{0};
    int32_t offset{0};
    bool    offset_valid{false};
    double  pos{0.0};     // rad
    double  vel{0.0};     // rad/s (measured)
    double  cmd{0.0};     // rad/s (commanded)
  };
  Wheel left_, right_;

  // ---- link health -------------------------------------------------------
  bool     have_frame_{false};
  uint64_t last_frame_ns_{0};
  uint16_t last_seq_{0};
  bool     seq_valid_{false};
  uint64_t dropped_frames_{0};
  uint64_t crc_errors_{0};
  uint16_t mcu_flags_{0};
};

}  // namespace minibot_hardware

#endif  // MINIBOT_HARDWARE__STEPPER_DIFFDRIVE_HPP_
