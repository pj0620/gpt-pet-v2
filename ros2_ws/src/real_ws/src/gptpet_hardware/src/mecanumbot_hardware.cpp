#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class YourMecanumHardwareInterface : public hardware_interface::SystemInterface {
public:
  YourMecanumHardwareInterface() = default;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // Initialize internal storage
    size_t n = info.joints.size();
    hw_positions_.resize(n, 0.0);
    hw_velocities_.resize(n, 0.0);
    hw_commands_.resize(n, 0.0);

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

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override {
    // Simulate encoder feedback or read from hardware
    for (size_t i = 0; i < hw_positions_.size(); ++i) {
      hw_positions_[i] += hw_velocities_[i] * 0.01; // fake position integration
      hw_velocities_[i] = hw_commands_[i]; // pretend motor velocity = command
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override {
    // Send hw_commands_ to motor driver
    // Example: serial.writeMotorSpeed(hw_commands_)
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
};

PLUGINLIB_EXPORT_CLASS(YourMecanumHardwareInterface, hardware_interface::SystemInterface)

