#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <algorithm>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gptpet_hardware {

class GptpetHardwareInterface : public hardware_interface::SystemInterface {
public:
  GptpetHardwareInterface() 
    : logger_(rclcpp::get_logger("GptpetHardwareInterface")), serial_fd_(-1) {
    RCLCPP_INFO(logger_, "GptpetHardwareInterface initialized");
  }

  ~GptpetHardwareInterface() {
    if (serial_fd_ >= 0) {
      close(serial_fd_);
    }
  }

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(logger_, "Failed to initialize hardware interface");
      return CallbackReturn::ERROR;
    }

    // Initialize internal storage
    size_t n = info.joints.size();
    hw_positions_.resize(n, 0.0);
    hw_velocities_.resize(n, 0.0);
    hw_commands_.resize(n, 0.0);
    
    // Initialize serial port
    if (!init_serial_port()) {
      RCLCPP_ERROR(logger_, "Failed to initialize serial port");
      return CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(logger_, "Hardware interface initialized with %zu joints", n);
    for (size_t i = 0; i < n; ++i) {
      RCLCPP_INFO(logger_, "Joint %zu: %s", i, info.joints[i].name.c_str());
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
      state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
    }
    return command_interfaces;
  }

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
    // Read serial data for motor speeds
    read_serial_data();
    
    // Update positions based on velocities (integration)
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
      hw_positions_[i] += hw_velocities_[i] * period.seconds();
    }
    
    // Only log periodically to avoid flooding the console
    read_count_++;
    if (read_count_ % 100 == 0) {  // Log every ~100 calls (assuming 100Hz control loop)
      RCLCPP_INFO(logger_, "Read state - Time: %.3f.%09ld", time.seconds(), time.nanoseconds());
      log_joint_states("Position", hw_positions_);
      log_joint_states("Velocity", hw_velocities_);
    }
    
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration &) override {
    // Send motor commands via serial port
    write_serial_command();
    
    // Log commands that are being sent to motor driver
    write_count_++;
    if (write_count_ % 100 == 0) {  // Log every ~100 calls
      RCLCPP_INFO(logger_, "Write command - Time: %.3f.%09ld", time.seconds(), time.nanoseconds());
      log_joint_states("Command", hw_commands_);
    }
    
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
  rclcpp::Logger logger_;
  unsigned int read_count_ = 0;
  unsigned int write_count_ = 0;
  int serial_fd_;
  std::vector<std::string> serial_ports_ = {"/dev/ttyACM0", "/dev/ttyACM1"};
  
  bool init_serial_port() {
    for (const auto& port : serial_ports_) {
      serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (serial_fd_ >= 0) {
        // Configure serial port
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
          RCLCPP_ERROR(logger_, "Error getting serial port attributes for %s", port.c_str());
          close(serial_fd_);
          serial_fd_ = -1;
          continue;
        }
        
        // Set baud rate to 115200
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        
        // Configure for 8N1
        tty.c_cflag &= ~PARENB;  // No parity
        tty.c_cflag &= ~CSTOPB;  // One stop bit
        tty.c_cflag &= ~CSIZE;   // Clear size bits
        tty.c_cflag |= CS8;      // 8 data bits
        
        // Disable flow control
        tty.c_cflag &= ~CRTSCTS;
        
        // Enable reading and ignore modem control lines
        tty.c_cflag |= CREAD | CLOCAL;
        
        // Set input mode (non-canonical, no echo, etc.)
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        
        // Set output mode (raw output)
        tty.c_oflag &= ~OPOST;
        
        // Set input mode (raw input)
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        
        // Set timeouts
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;  // 0.1 seconds
        
        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
          RCLCPP_ERROR(logger_, "Error setting serial port attributes for %s", port.c_str());
          close(serial_fd_);
          serial_fd_ = -1;
          continue;
        }
        
        RCLCPP_INFO(logger_, "Successfully opened serial port: %s", port.c_str());
        return true;
      } else {
        RCLCPP_WARN(logger_, "Unable to open serial port: %s", port.c_str());
      }
    }
    
    RCLCPP_ERROR(logger_, "All serial ports unavailable");
    return false;
  }
  
  uint8_t float_to_byte(double f) {
    double value = std::max(-2.0, std::min(2.0, f));  // Clamp to [-2, 2] range
    int int_val = static_cast<int>(std::round(value * 127.0 / 2.0));
    int_val = std::max(-127, std::min(127, int_val));
    
    uint8_t sign_bit = (int_val < 0) ? 0x80 : 0x00;
    uint8_t magnitude = static_cast<uint8_t>(std::abs(int_val)) & 0x7F;
    return sign_bit | magnitude;
  }
  
  double byte_to_float(uint8_t b) {
    double val = 2.0 * static_cast<double>(b & 0x7F) / 127.0;
    if (b & 0x80) {
      val = -val;
    }
    return val;
  }
  
  void read_serial_data() {
    if (serial_fd_ < 0) return;
    
    uint8_t buffer[256];
    ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));
    
    if (bytes_read > 0) {
      // Look for motor data packets starting with 'M'
      for (ssize_t i = 0; i < bytes_read; ++i) {
        if (buffer[i] == 'M' && i + 4 < bytes_read) {
          // Parse motor velocities (assuming 4 motors)
          if (hw_velocities_.size() >= 4) {
            hw_velocities_[0] = byte_to_float(buffer[i + 1]);  // left_1
            hw_velocities_[1] = byte_to_float(buffer[i + 2]);  // left_2
            hw_velocities_[2] = byte_to_float(buffer[i + 3]);  // right_1
            hw_velocities_[3] = byte_to_float(buffer[i + 4]);  // right_2
          }
          break;
        }
      }
    }
  }
  
  void write_serial_command() {
    if (serial_fd_ < 0) return;
    
    uint8_t command[6];  // 'V' + 4 motor speed bytes + null terminator
    command[0] = 'V';
    
    // Ensure we have at least 4 motor commands
    size_t num_motors = std::min(hw_commands_.size(), static_cast<size_t>(4));
    for (size_t i = 0; i < num_motors; ++i) {
      command[i + 1] = float_to_byte(hw_commands_[i]);
    }
    
    // Fill remaining slots with zero if we have fewer than 4 motors
    for (size_t i = num_motors; i < 4; ++i) {
      command[i + 1] = float_to_byte(0.0);
    }
    
    // Log the values being written
    std::stringstream ss;
    ss << "Writing serial command: ";
    for (int i = 0; i < 5; ++i) {
      ss << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(command[i]) << " ";
    }
    RCLCPP_INFO(logger_, "%s", ss.str().c_str());

    // Log hw_commands_ values
    std::stringstream cmd_ss;
    cmd_ss << "hw_commands_: ";
    for (size_t i = 0; i < hw_commands_.size(); ++i) {
      cmd_ss << std::fixed << std::setprecision(3) << hw_commands_[i];
      if (i < hw_commands_.size() - 1) {
      cmd_ss << ", ";
      }
    }
    RCLCPP_INFO(logger_, "%s", cmd_ss.str().c_str());

    ssize_t bytes_written = ::write(serial_fd_, command, 5);
    if (bytes_written != 5) {
      RCLCPP_WARN(logger_, "Failed to write complete command to serial port");
    }
  }
  
  void log_joint_states(const std::string& label, const std::vector<double>& values) {
    std::stringstream ss;
    ss << label << " values: ";
    for (size_t i = 0; i < values.size(); ++i) {
      ss << std::fixed << std::setprecision(3) << info_.joints[i].name << "=" << values[i];
      if (i < values.size() - 1) {
        ss << ", ";
      }
    }
    RCLCPP_INFO(logger_, "%s", ss.str().c_str());
  }
};

}  // namespace gptpet_hardware

PLUGINLIB_EXPORT_CLASS(gptpet_hardware::GptpetHardwareInterface, hardware_interface::SystemInterface)

