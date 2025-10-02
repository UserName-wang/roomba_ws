#ifndef ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_INTERFACE_HPP_
#define ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "roomba_hardware_interface/visibility_control.h"
#include "roomba_hardware_interface/roomba_hardware_driver.hpp"

namespace roomba_hardware_interface
{
class RoombaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoombaHardwareInterface)

  ROOMBA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ROOMBA_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROOMBA_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROOMBA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ROOMBA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  ROOMBA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROOMBA_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Roomba hardware communication
  std::unique_ptr<RoombaHardware> roomba_;

  // Store the hardware information
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
  std::string port_;
  int baud_rate_;
  
  // Smooth velocity control
  std::vector<double> current_wheel_velocities_;  // 当前实际发送给轮子的速度
  double max_acceleration_;  // 最大加速度 (rad/s^2)
  double wheel_radius_;      // 轮子半径 (m)
};

}  // namespace roomba_hardware_interface

#endif  // ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_INTERFACE_HPP_