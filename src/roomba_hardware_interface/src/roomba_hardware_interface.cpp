#include "roomba_hardware_interface/roomba_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace roomba_hardware_interface
{
hardware_interface::CallbackReturn RoombaHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize vectors for commands and states
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  current_wheel_velocities_.resize(info_.joints.size(), 0.0);

  // Get parameters
  port_ = info_.hardware_parameters["port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  
  // Get acceleration parameter or use default value
  if (info_.hardware_parameters.count("max_acceleration") > 0) {
    max_acceleration_ = std::stod(info_.hardware_parameters["max_acceleration"]);
  } else {
    max_acceleration_ = 0.5; // 默认最大加速度 0.5 rad/s^2
  }
  
  // Get wheel radius parameter or use default value
  if (info_.hardware_parameters.count("wheel_radius") > 0) {
    wheel_radius_ = std::stod(info_.hardware_parameters["wheel_radius"]);
  } else {
    wheel_radius_ = 0.03; // 默认轮子半径 3cm
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoombaHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoombaHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RoombaHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create Roomba hardware instance
  roomba_ = std::make_unique<RoombaHardware>(port_, baud_rate_);
  
  // Connect to Roomba
  if (!roomba_->connect()) {
    RCLCPP_FATAL(rclcpp::get_logger("RoombaHardwareInterface"), "Failed to connect to Roomba");
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Start Roomba and set to safe mode
  roomba_->start();
  roomba_->safe_mode();
  
  // Initialize state and command values
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = 0.0;
    hw_commands_[i] = 0.0;
    current_wheel_velocities_[i] = 0.0;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), "Successfully activated Roomba hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoombaHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (roomba_) {
    roomba_->stop();
    roomba_->disconnect();
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), "Successfully deactivated Roomba hardware interface");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoombaHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!roomba_) {
    return hardware_interface::return_type::ERROR;
  }
  
  // In a real implementation, we would read sensor data from the Roomba
  // and update the state interfaces accordingly
  // For now, we'll just increment positions based on commands
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] += hw_commands_[i] * 0.01; // Simple integration
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoombaHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!roomba_) {
    return hardware_interface::return_type::ERROR;
  }
  
  // Apply smooth acceleration control
  double dt = period.seconds();
  if (dt <= 0) {
    dt = 0.01; // Default to 10ms if period is invalid
  }
  
  // Calculate maximum velocity change allowed in this time step
  double max_velocity_change = max_acceleration_ * dt;
  
  // Smooth velocity transition for each wheel
  for (uint i = 0; i < hw_commands_.size(); i++) {
    if (!std::isnan(hw_commands_[i])) {
      // Calculate desired velocity change
      double desired_velocity_change = hw_commands_[i] - current_wheel_velocities_[i];
      
      // Limit velocity change to maximum allowed
      if (std::abs(desired_velocity_change) > max_velocity_change) {
        if (desired_velocity_change > 0) {
          current_wheel_velocities_[i] += max_velocity_change;
        } else {
          current_wheel_velocities_[i] -= max_velocity_change;
        }
      } else {
        // 如果变化量在允许范围内，则直接应用
        current_wheel_velocities_[i] = hw_commands_[i];
      }
    }
  }
  
  // Convert command interfaces to Roomba velocity commands
  // For a differential drive robot, we expect 2 joints (left and right wheels)
  if (current_wheel_velocities_.size() >= 2) {
    // Convert from rad/s to mm/s
    // Wheel radius is 0.03m = 30mm
    int16_t right_velocity = static_cast<int16_t>(current_wheel_velocities_[0] * wheel_radius_ * 1000.0); // right wheel
    int16_t left_velocity = static_cast<int16_t>(current_wheel_velocities_[1] * wheel_radius_ * 1000.0);  // left wheel
    
    // Limit velocities to Roomba's capabilities (-500 to 500 mm/s)
    right_velocity = std::max(static_cast<int16_t>(-500), std::min(static_cast<int16_t>(500), right_velocity));
    left_velocity = std::max(static_cast<int16_t>(-500), std::min(static_cast<int16_t>(500), left_velocity));
    
    // Send command to Roomba
    roomba_->drive_direct(right_velocity, left_velocity);
  }
  
  return hardware_interface::return_type::OK;
}

}  // namespace roomba_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  roomba_hardware_interface::RoombaHardwareInterface, hardware_interface::SystemInterface)