#pragma once

#include <string>
#include <vector>
#include <memory>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"

namespace diff {

// Forward declarations to keep header clean and hide implementation details
class WiFiClientWrapper;
class SerialPortWrapper;
class MotorCharacterization;
class SimplePID;

class DiffBotSystemHardware : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware)
  
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Explicitly default destructor is REQUIRED when using std::unique_ptr with forward-declared types
  ~DiffBotSystemHardware() override = default;

private:
  hardware_interface::HardwareInfo info_;
  
  std::unique_ptr<WiFiClientWrapper> wifi_client_;
  std::string esp32_ip_{"ip"}; // Run the ESP32 code in the Arduino IDE (or any other compatible IDE), then check the Serial Monitor for the ESP32’s IP address and enter that IP address here.
  int esp32_port_{80};
  
  std::unique_ptr<SerialPortWrapper> serial_client_;
  std::string serial_port_name_{"/dev/ttyUSB0"};
  int serial_baud_{115200};
  
  std::string wifi_read_buffer_{""};
  
  double wheel_radius_{0.033};       
  double wheel_separation_{0.128};   
  int ticks_per_rev_{20};
  double max_wheel_vel_mps_{1.18};   
  long left_ticks_{0}; long right_ticks_{0};
  long prev_left_ticks_{0}; long prev_right_ticks_{0};
  
  double left_position_{0.0}; double right_position_{0.0};       
  double left_velocity_{0.0}; double right_velocity_{0.0};       // Stored in rad/s
  double left_velocity_command_{0.0}; double right_velocity_command_{0.0};

  // IMU STATE VARIABLES
  double imu_orientation_x_{0.0};
  double imu_orientation_y_{0.0};
  double imu_orientation_z_{0.0};
  double imu_orientation_w_{1.0};
  double imu_angular_velocity_x_{0.0};
  double imu_angular_velocity_y_{0.0};
  double imu_angular_velocity_z_{0.0};
  double imu_linear_acceleration_x_{0.0};
  double imu_linear_acceleration_y_{0.0};
  double imu_linear_acceleration_z_{0.0};
  double prev_yaw_{0.0};

  rclcpp::Time last_valid_parse_{0, 0, RCL_ROS_TIME}; 
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME}; 

  // CONTROL LOOP COMPONENTS
  std::unique_ptr<MotorCharacterization> left_motor_char_;
  std::unique_ptr<MotorCharacterization> right_motor_char_;
  std::unique_ptr<SimplePID> left_pid_;
  std::unique_ptr<SimplePID> right_pid_;
};

} // namespace diff