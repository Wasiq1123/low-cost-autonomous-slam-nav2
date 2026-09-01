#include "diff/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <vector>
#include <cstdio>
#include <iomanip>
#include <algorithm>
#include <fcntl.h>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <asio.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/serial_port.hpp>
#include <asio/serial_port_base.hpp>

namespace diff {

class SimplePID {
public:
    double Kp, Ki, Kd;
    double integral = 0.0;
    double prev_error = 0.0;

    SimplePID(double p, double i, double d) : Kp(p), Ki(i), Kd(d) {}

    double compute(double error, double dt) {
        if (dt <= 0.0) return 0.0;
        if (std::abs(error) < 0.01) {
            integral = 0.0;
        } else {
            integral += error * dt;
            integral = std::clamp(integral, -100.0, 100.0); 
        }
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
        return std::clamp(output, -255.0, 255.0);
    }
    void reset() { integral = 0.0; prev_error = 0.0; }
};

// Quintic Spline for smooth motor characterization
class QuinticSpline {
private:
    struct Segment {
        double x0, x1;
        double a0, a1, a2, a3, a4, a5; // coefficients: a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    };
    std::vector<Segment> segments_;
    std::vector<double> knots_;

public:
    QuinticSpline() = default;

    void build(const std::vector<std::pair<double, double>>& data) {
        if (data.size() < 2) return;

        knots_.clear();
        segments_.clear();
        
        for (const auto& p : data) {
            knots_.push_back(p.first);
        }

        for (size_t i = 0; i < data.size() - 1; ++i) {
            Segment seg;
            seg.x0 = data[i].first;
            seg.x1 = data[i+1].first;
            
            double h = seg.x1 - seg.x0;
            double y0 = data[i].second;
            double y1 = data[i+1].second;
            
            double dy = y1 - y0;
            
            // Coefficients for quintic polynomial
            seg.a0 = y0;
            seg.a1 = dy / h;  // First derivative approximation
            seg.a2 = 0.0;     // Zero second derivative at start (natural boundary)
            seg.a3 = 10.0 * (dy - seg.a1 * h) / (h * h * h);
            seg.a4 = -15.0 * (dy - seg.a1 * h) / (h * h * h * h);
            seg.a5 = 6.0 * (dy - seg.a1 * h) / (h * h * h * h * h);
            
            segments_.push_back(seg);
        }
    }

    double evaluate(double x) const {
        if (segments_.empty()) return 0.0;
        
        if (x <= segments_.front().x0) return segments_.front().a0;
        if (x >= segments_.back().x1) {
            const auto& last = segments_.back();
            double t = last.x1 - last.x0;
            return last.a0 + last.a1*t + last.a2*t*t + last.a3*t*t*t + last.a4*t*t*t*t + last.a5*t*t*t*t*t;
        }

        for (const auto& seg : segments_) {
            if (x >= seg.x0 && x <= seg.x1) {
                double t = x - seg.x0;
                return seg.a0 + seg.a1*t + seg.a2*t*t + seg.a3*t*t*t + seg.a4*t*t*t*t + seg.a5*t*t*t*t*t;
            }
        }
        
        return 0.0;
    }
};

class MotorCharacterization {
private:
    std::vector<std::pair<double, double>> pwm_to_vel_;
    std::vector<std::pair<double, double>> vel_to_pwm_;
    QuinticSpline pwm_to_vel_spline_;
    QuinticSpline vel_to_pwm_spline_;

public:
    MotorCharacterization(std::vector<std::pair<double, double>> pwm_vel_map) {
        if (pwm_vel_map.empty() || pwm_vel_map.front().first > 0.0) {
            pwm_vel_map.insert(pwm_vel_map.begin(), {0.0, 0.0});
        }
        std::sort(pwm_vel_map.begin(), pwm_vel_map.end(), 
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        pwm_to_vel_ = pwm_vel_map;

        vel_to_pwm_.reserve(pwm_vel_map.size());
        for (const auto& pair : pwm_vel_map) {
            vel_to_pwm_.emplace_back(pair.second, pair.first);
        }
        std::sort(vel_to_pwm_.begin(), vel_to_pwm_.end(), 
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // Build quintic splines for both directions
        pwm_to_vel_spline_.build(pwm_to_vel_);
        vel_to_pwm_spline_.build(vel_to_pwm_);
    }

    double getVelocity(double pwm) const {
        double abs_pwm = std::abs(pwm);
        double vel = pwm_to_vel_spline_.evaluate(abs_pwm);
        return (pwm < 0.0) ? -vel : vel;
    }

    double getPWM(double velocity) const {
        if (velocity <= vel_to_pwm_.front().first) return vel_to_pwm_.front().second;
        if (velocity >= vel_to_pwm_.back().first) return vel_to_pwm_.back().second;
        return vel_to_pwm_spline_.evaluate(velocity);
    }
};

class WiFiClientWrapper {
public:
  WiFiClientWrapper() : socket_(io_context_), connected_(false) {}
  bool connect(const std::string & ip, uint16_t port) {
    ip_ = ip; port_ = port; 
    try {
      if (socket_.is_open()) { asio::error_code ec; socket_.close(ec); }
      asio::ip::tcp::resolver resolver(io_context_);
      auto endpoints = resolver.resolve(ip, std::to_string(port));
      asio::error_code ec; 
      asio::connect(socket_, endpoints, ec);
      if (ec) { connected_ = false; return false; }
      socket_.set_option(asio::ip::tcp::no_delay(true));
      socket_.non_blocking(true); 
      connected_ = true;
      RCLCPP_INFO(rclcpp::get_logger("WiFiClient"), "✅ WiFi Connected to %s:%d", ip.c_str(), port);
      return true;
    } catch (...) { connected_ = false; return false; }
  }
  void ensure_connected() {
    if (!connected_) {
      static auto last_warn = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_warn).count() > 2) {
        RCLCPP_WARN(rclcpp::get_logger("WiFiClient"), "⚠️ WiFi Disconnected! Attempting Reconnect...");
        last_warn = now;
      }
      connect(ip_, port_);
    }
  }
  bool send(const std::string & data) {
    if (!connected_) return false;
    try { 
      asio::error_code ec; 
      asio::write(socket_, asio::buffer(data), ec); 
      if (ec) { connected_ = false; return false; } 
      return true; 
    } catch (...) { connected_ = false; return false; }
  }
  std::string read_available() {
    if (!connected_) return "";
    std::string result; char buf[256]; asio::error_code ec;
    while (true) {
      size_t n = socket_.read_some(asio::buffer(buf, sizeof(buf)), ec);
      if (ec == asio::error::would_block || ec == asio::error::try_again) break;
      if (ec) { connected_ = false; break; }
      if (n > 0) result.append(buf, n); else break;
    }
    return result;
  }
  bool isConnected() const { return connected_; }
  void disconnect() { if (connected_) { asio::error_code ec; socket_.close(ec); connected_ = false; } }
private:
  asio::io_context io_context_; 
  asio::ip::tcp::socket socket_; 
  bool connected_ = false;
  std::string ip_;
  uint16_t port_;
};

class SerialPortWrapper {
public:
  SerialPortWrapper() : port_(io_context_) {}
  bool connect(const std::string& device, int baudrate) {
    try {
      if (port_.is_open()) { asio::error_code ec; port_.close(ec); }
      port_.open(device);
      port_.set_option(asio::serial_port_base::baud_rate(baudrate));
      port_.set_option(asio::serial_port_base::character_size(8));
      port_.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
      port_.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
      port_.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
      int fd = port_.native_handle(); fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
      connected_ = true; return true;
    } catch (...) { connected_ = false; return false; }
  }
  bool send(const std::string& data) { if (!connected_) return false; try { asio::write(port_, asio::buffer(data)); return true; } catch (...) { connected_ = false; return false; } }
  std::string read_available() {
    if (!connected_) return "";
    std::string result; char buf[128]; asio::error_code ec;
    while (true) {
      size_t n = port_.read_some(asio::buffer(buf, sizeof(buf)), ec);
      if (ec == asio::error::would_block || ec == asio::error::try_again) break;
      if (ec) { connected_ = false; break; }
      if (n > 0) result.append(buf, n); else break;
    }
    return result;
  }
  bool isConnected() const { return connected_; }
  void disconnect() { if (connected_) { asio::error_code ec; port_.close(ec); connected_ = false; } }
private:
  asio::io_context io_context_; asio::serial_port port_; bool connected_ = false;
};

hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(const hardware_interface::HardwareInfo & info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) return hardware_interface::CallbackReturn::ERROR;
  info_ = info;
  auto get_param = [this](const std::string & key, const std::string & default_val) -> std::string {
    auto it = info_.hardware_parameters.find(key); 
    return (it != info_.hardware_parameters.end()) ? it->second : default_val;
  };
  esp32_ip_ = get_param("esp32_ip", "192.168.43.55"); 
  esp32_port_ = std::stoi(get_param("esp32_port", "80"));
  wheel_radius_ = std::stod(get_param("wheel_radius", "0.033")); 
  wheel_separation_ = std::stod(get_param("wheel_separation", "0.128")); 
  ticks_per_rev_ = std::stoi(get_param("ticks_per_rev", "20"));
  max_wheel_vel_mps_ = std::stod(get_param("max_wheel_vel_mps", "1.18"));
  if (ticks_per_rev_ <= 0) ticks_per_rev_ = 20;

  // NEW UNDER-LOAD CHARACTERIZATION (extracted from actual robot driving logs)
  // Format: {PWM, velocity_mps}
  left_motor_char_ = std::make_unique<MotorCharacterization>(std::vector<std::pair<double, double>>{
      {0.0, 0.0},      // Deadzone
      {25.0, 0.0},     // Below friction threshold
      {36.0, 0.12},    // Measured under load
      {41.0, 0.13},    // Measured under load
      {53.0, 0.20},    // Measured under load
      {62.0, 0.25},    // Measured under load
      {74.0, 0.32},    // Measured under load
      {90.0, 0.41},    // Measured under load
      {100.0, 0.45},   // Extrapolated for headroom
      {120.0, 0.52},   // Extrapolated
      {150.0, 0.62},   // Extrapolated
      {180.0, 0.71},   // Extrapolated
      {210.0, 0.79},   // Extrapolated
      {240.0, 0.86},   // Extrapolated
      {255.0, 0.90}    // Max PWM
  });

  right_motor_char_ = std::make_unique<MotorCharacterization>(std::vector<std::pair<double, double>>{
      {0.0, 0.0},      // Deadzone
      {25.0, 0.0},     // Below friction threshold
      {34.0, 0.11},    // Measured under load
      {40.0, 0.14},    // Measured under load
      {51.0, 0.21},    // Measured under load
      {60.0, 0.26},    // Measured under load
      {72.0, 0.31},    // Measured under load
      {86.0, 0.40},    // Measured under load
      {100.0, 0.45},   // Extrapolated for headroom
      {120.0, 0.52},   // Extrapolated
      {150.0, 0.62},   // Extrapolated
      {180.0, 0.71},   // Extrapolated
      {210.0, 0.79},   // Extrapolated
      {240.0, 0.86},   // Extrapolated
      {255.0, 0.90}    // Max PWM
  });

  // PID Gains: Both wheels use same gains (feedforward handles asymmetry)
  left_pid_  = std::make_unique<SimplePID>(5.0, 3.0, 0.0); 
  right_pid_ = std::make_unique<SimplePID>(5.0, 3.0, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_configure(const rclcpp_lifecycle::State &) { return hardware_interface::CallbackReturn::SUCCESS; }

hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(const rclcpp_lifecycle::State &) {
  wifi_client_ = std::make_unique<WiFiClientWrapper>(); 
  serial_client_ = std::make_unique<SerialPortWrapper>();
  bool wifi_ok = wifi_client_->connect(esp32_ip_, esp32_port_);
  if (!wifi_ok) { RCLCPP_ERROR(rclcpp::get_logger("DiffBotHardware"), "❌ WiFi Failed!"); return hardware_interface::CallbackReturn::ERROR; }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &) {
  if (wifi_client_) wifi_client_->disconnect(); 
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_POSITION, &left_position_);
  state_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_velocity_);
  state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_POSITION, &right_position_);
  state_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_velocity_);
  state_interfaces.emplace_back("imu_sensor", "orientation.x", &imu_orientation_x_);
  state_interfaces.emplace_back("imu_sensor", "orientation.y", &imu_orientation_y_);
  state_interfaces.emplace_back("imu_sensor", "orientation.z", &imu_orientation_z_);
  state_interfaces.emplace_back("imu_sensor", "orientation.w", &imu_orientation_w_);
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.x", &imu_angular_velocity_x_);
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.y", &imu_angular_velocity_y_);
  state_interfaces.emplace_back("imu_sensor", "angular_velocity.z", &imu_angular_velocity_z_);
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.x", &imu_linear_acceleration_x_);
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.y", &imu_linear_acceleration_y_);
  state_interfaces.emplace_back("imu_sensor", "linear_acceleration.z", &imu_linear_acceleration_z_);
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back("left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &left_velocity_command_);
  command_interfaces.emplace_back("right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &right_velocity_command_);
  return command_interfaces;
}

hardware_interface::return_type DiffBotSystemHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
  double left_cmd_rad_s = left_velocity_command_;
  double right_cmd_rad_s = right_velocity_command_;

  if (std::isnan(left_cmd_rad_s) || std::isinf(left_cmd_rad_s)) left_cmd_rad_s = 0.0;
  if (std::isnan(right_cmd_rad_s) || std::isinf(right_cmd_rad_s)) right_cmd_rad_s = 0.0;

  // 1. Convert commands to m/s
  double left_cmd_mps = left_cmd_rad_s * wheel_radius_;
  double right_cmd_mps = right_cmd_rad_s * wheel_radius_;

  // 2. Proportional Velocity Saturation
  double max_abs_vel = std::max({std::abs(left_cmd_mps), std::abs(right_cmd_mps)});
  if (max_abs_vel > max_wheel_vel_mps_) {
    double scale = max_wheel_vel_mps_ / max_abs_vel;
    left_cmd_mps *= scale;
    right_cmd_mps *= scale;
  }

  // 3. Read actual velocity in m/s
  double left_actual_mps = left_velocity_ * wheel_radius_;
  double right_actual_mps = right_velocity_ * wheel_radius_;

  // 4. PID Feedback Calculation
  double dt = period.seconds();
  if (dt <= 0.0) dt = 0.01; 
  
  double left_error = left_cmd_mps - left_actual_mps;
  double right_error = right_cmd_mps - right_actual_mps;

  double left_pid_out = left_pid_->compute(left_error, dt);
  double right_pid_out = right_pid_->compute(right_error, dt);

  // 5. Feedforward Calculation using Quintic Spline
  double left_ff_pwm = left_motor_char_->getPWM(std::abs(left_cmd_mps));
  double right_ff_pwm = right_motor_char_->getPWM(std::abs(right_cmd_mps));
  
  if (left_cmd_mps < 0.0) left_ff_pwm = -left_ff_pwm;
  if (right_cmd_mps < 0.0) right_ff_pwm = -right_ff_pwm;

  // 6. Sum Feedforward + Feedback and Clamp
  double left_final_pwm = std::clamp(left_ff_pwm + left_pid_out, -255.0, 255.0);
  double right_final_pwm = std::clamp(right_ff_pwm + right_pid_out, -255.0, 255.0);

  // ENHANCED LOGGING: Explicitly shows Target (Cmd), Actual (Odom), and the Difference (Err)
  RCLCPP_INFO_THROTTLE(rclcpp::get_logger("DiffBotHardware"), steady_clock_, 1000, 
      "📊 VELOCITY TRACKING -> "
      "L: Cmd=%.2f, Odom=%.2f, Err=%.2f m/s | "
      "R: Cmd=%.2f, Odom=%.2f, Err=%.2f m/s | "
      "PWM -> L: %d, R: %d", 
      left_cmd_mps, left_actual_mps, left_error,
      right_cmd_mps, right_actual_mps, right_error,
      static_cast<int>(std::round(left_final_pwm)),
      static_cast<int>(std::round(right_final_pwm)));

  // 7. Send to ESP32 (Integer only)
  std::ostringstream oss;
  oss << "PL:" << static_cast<int>(std::round(left_final_pwm)) 
      << ",PR:" << static_cast<int>(std::round(right_final_pwm)) << "\n";
  
  if (wifi_client_) {
    wifi_client_->ensure_connected();
    if (wifi_client_->isConnected()) wifi_client_->send(oss.str());
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
  if (wifi_client_) {
    wifi_client_->ensure_connected();
    if (wifi_client_->isConnected()) wifi_read_buffer_ += wifi_client_->read_available();
  }

  auto extract_latest_line = [](std::string& buffer) -> std::string {
    size_t last_nl = buffer.find_last_of('\n');
    if (last_nl == std::string::npos) return "";
    if (last_nl == 0) { buffer.erase(0, 1); return ""; }
    size_t prev_nl = buffer.find_last_of('\n', last_nl - 1);
    std::string line;
    if (prev_nl != std::string::npos) line = buffer.substr(prev_nl + 1, last_nl - prev_nl - 1);
    else line = buffer.substr(0, last_nl);
    buffer.erase(0, last_nl + 1);
    return line;
  };

  std::string line_to_parse = extract_latest_line(wifi_read_buffer_);

  if (!line_to_parse.empty()) {
    long l_ticks = 0, r_ticks = 0;
    float yaw_rad = 0.0f, vl_mps = 0.0f, vr_mps = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    
    int parsed = sscanf(line_to_parse.c_str(), 
        "L:%ld,R:%ld,Y:%f,VL:%f,VR:%f,AX:%f,AY:%f,AZ:%f,GX:%f,GY:%f,GZ:%f", 
        &l_ticks, &r_ticks, &yaw_rad, &vl_mps, &vr_mps, 
        &ax, &ay, &az, &gx, &gy, &gz);

    if (parsed == 11) {
      left_ticks_ = l_ticks; 
      right_ticks_ = r_ticks; 
      left_velocity_ = vl_mps / wheel_radius_; 
      right_velocity_ = vr_mps / wheel_radius_;
      last_valid_parse_ = time; 
      
      double half_yaw = yaw_rad * 0.5;
      imu_orientation_x_ = 0.0;
      imu_orientation_y_ = 0.0;
      imu_orientation_z_ = std::sin(half_yaw);
      imu_orientation_w_ = std::cos(half_yaw);
      
      imu_linear_acceleration_x_ = ax;
      imu_linear_acceleration_y_ = ay;
      imu_linear_acceleration_z_ = az;
      imu_angular_velocity_x_ = gx;
      imu_angular_velocity_y_ = gy;
      imu_angular_velocity_z_ = gz; 
      
      RCLCPP_INFO_THROTTLE(rclcpp::get_logger("DiffBotHardware"), steady_clock_, 1000, "📥 ROS2 RECEIVED: %s", line_to_parse.c_str());
    }
  }

  // Stale data safety check
  if (last_valid_parse_.nanoseconds() != 0) {
    double age = (time - last_valid_parse_).seconds();
    if (age > 0.5) {   // Tolerate WiFi jitter
      if (left_velocity_ != 0.0 || right_velocity_ != 0.0) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("DiffBotHardware"), steady_clock_, 1000, "⚠️ Stale velocity — zeroing after %.3fs", age);
      }
      left_velocity_ = 0.0; right_velocity_ = 0.0;
    }
  }

  long delta_left = left_ticks_ - prev_left_ticks_; 
  long delta_right = right_ticks_ - prev_right_ticks_;
  prev_left_ticks_ = left_ticks_; prev_right_ticks_ = right_ticks_;
  
  double circumference = 2.0 * M_PI * wheel_radius_;
  left_position_ += (circumference * static_cast<double>(delta_left) / static_cast<double>(ticks_per_rev_)) / wheel_radius_;
  right_position_ += (circumference * static_cast<double>(delta_right) / static_cast<double>(ticks_per_rev_)) / wheel_radius_;
  
  return hardware_interface::return_type::OK;
}

}  // namespace diff

PLUGINLIB_EXPORT_CLASS(diff::DiffBotSystemHardware, hardware_interface::SystemInterface)