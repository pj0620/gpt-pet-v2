#ifndef YOUR_MECANUM_HARDWARE_INTERFACE_HPP_
#define YOUR_MECANUM_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

class YourMecanumHardwareInterface : public hardware_interface::SystemInterface {
public:
  YourMecanumHardwareInterface();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
};

#endif  // YOUR_MECANUM_HARDWARE_INTERFACE_HPP_
