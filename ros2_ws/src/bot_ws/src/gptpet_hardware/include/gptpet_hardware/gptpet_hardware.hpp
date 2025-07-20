#ifndef GPTPET_HARDWARE_INTERFACE_HPP
#define GPTPET_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <pluginlib/class_list_macros.h>

namespace gptpet_hardware {

  class GptpetHardwareInterface : public hardware_interface::SystemInterface {
    public:
      GptpetHardwareInterface();

      CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
      hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
      hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    private:
      std::vector<double> hw_positions_, hw_velocities_, hw_commands_;
  };

}  // namespace gptpet_hardware

#endif  // GPTPET_HARDWARE_INTERFACE_HPP
