#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <iostream>
#include <iomanip>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace gptpet_hardware {

class GptpetHardwareInterface : public hardware_interface::SystemInterface {
public:
  GptpetHardwareInterface() 
    : logger_(rclcpp::get_logger("GptpetHardwareInterface")) {
    RCLCPP_INFO(logger_, "GptpetHardwareInterface initialized");
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
    // Simulate encoder feedback or read from hardware
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
      hw_positions_[i] += hw_velocities_[i] * period.seconds(); // fake position integration
      hw_velocities_[i] = hw_commands_[i]; // pretend motor velocity = command
    }
    
    // Only log periodically to avoid flooding the console
    read_count_++;
    if (read_count_ % 100 == 0) {  // Log every ~100 calls (assuming 100Hz control loop)
      RCLCPP_INFO(logger_, "Read state - Time: %d.%09d", time.seconds(), time.nanoseconds());
      log_joint_states("Position", hw_positions_);
      log_joint_states("Velocity", hw_velocities_);
    }
    
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration &) override {
    // Log commands that would be sent to motor driver
    RCLCPP_INFO(logger_, "Write command - Time: %d.%09d", time.seconds(), time.nanoseconds());
    log_joint_states("Command", hw_commands_);
    
    // In a real implementation, we would send hw_commands_ to motor driver
    // Example: serial.writeMotorSpeed(hw_commands_)
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
  rclcpp::Logger logger_;
  unsigned int read_count_ = 0;
  
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

