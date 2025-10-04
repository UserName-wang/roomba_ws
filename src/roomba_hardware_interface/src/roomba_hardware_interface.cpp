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
hardware_interface::return_type RoombaHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info)
{
  info_ = info;
  
  // Initialize vectors for commands and states
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  current_wheel_velocities_.resize(info_.joints.size(), 0.0);

  // Get parameters
  port_ = info_.hardware_parameters.at("port");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  
  // Get acceleration parameter or use default value
  if (info_.hardware_parameters.count("max_acceleration") > 0) {
    max_acceleration_ = std::stod(info_.hardware_parameters.at("max_acceleration"));
  } else {
    max_acceleration_ = 0.5; // 默认最大加速度 0.5 rad/s^2
  }
  
  // Get wheel radius parameter or use default value
  if (info_.hardware_parameters.count("wheel_radius") > 0) {
    wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
  } else {
    wheel_radius_ = 0.03; // 默认轮子半径 3cm
  }

  return hardware_interface::return_type::OK;
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

hardware_interface::return_type RoombaHardwareInterface::start()
{
  // Create ROS2 node for publishers
  node_ = std::make_shared<rclcpp::Node>("roomba_hardware_interface");
  
  // Create publishers for sensor data
  battery_state_publisher_ = node_->create_publisher<sensor_msgs::msg::BatteryState>("~/battery_state", 10);
  wheel_speed_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("~/wheel_speed", 10);
  charging_state_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("~/charging_state", 10);
  
  // Create Roomba hardware instance
  roomba_ = std::make_unique<RoombaHardware>(port_, baud_rate_);
  
  // Connect to Roomba
  if (!roomba_->connect()) {
    RCLCPP_FATAL(rclcpp::get_logger("RoombaHardwareInterface"), "Failed to connect to Roomba");
    return hardware_interface::return_type::ERROR;
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
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoombaHardwareInterface::stop()
{
  if (roomba_) {
    roomba_->stop();
    roomba_->disconnect();
  }
  
  // Reset publishers
  battery_state_publisher_.reset();
  wheel_speed_publisher_.reset();
  charging_state_publisher_.reset();
  node_.reset();
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardwareInterface"), "Successfully deactivated Roomba hardware interface");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoombaHardwareInterface::read()
{
  if (!roomba_) {
    RCLCPP_ERROR(node_->get_logger(), "Roomba hardware interface not initialized");
    return hardware_interface::return_type::ERROR;
  }
  
  try {
    // Read sensor data from Roomba
    uint16_t battery_charge = roomba_->read_battery_charge();
    uint16_t battery_capacity = roomba_->read_battery_capacity();
    int8_t temperature = roomba_->read_temperature();
    uint8_t charging_state = roomba_->read_charging_state();
    
    // Create and publish battery state message
    auto battery_msg = std::make_unique<sensor_msgs::msg::BatteryState>();
    battery_msg->header.stamp = node_->now();
    battery_msg->voltage = 12.0f; // Nominal voltage for Roomba
    battery_msg->current = 0.0f; // Not directly available
    battery_msg->charge = static_cast<float>(battery_charge) / 1000.0f; // mAh to Ah
    battery_msg->capacity = static_cast<float>(battery_capacity) / 1000.0f; // mAh to Ah
    battery_msg->temperature = static_cast<float>(temperature);
    battery_msg->percentage = static_cast<float>(battery_charge) / static_cast<float>(battery_capacity);
    
    // Map charging state to power supply status
    switch (charging_state) {
      case 0: // Not charging
        battery_msg->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
        break;
      case 1: // Reconditioning charging
      case 2: // Full charging
      case 3: // Trickle charging
        battery_msg->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        break;
      case 4: // Waiting
        battery_msg->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
        break;
      case 5: // Charging fault condition
        battery_msg->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
        break;
      default:
        battery_msg->power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
    }
    
    battery_msg->power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
    battery_msg->power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    battery_msg->present = true;
    
    // Publish battery state
    battery_state_publisher_->publish(std::move(battery_msg));
    
    // Update velocity commands from topics
    // These are handled by the subscription callbacks
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Exception in read: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoombaHardwareInterface::write()
{
  if (!roomba_) {
    return hardware_interface::return_type::ERROR;
  }
  
  // Apply smooth acceleration control
  double dt = 0.01; // Default to 10ms
  
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
std::string RoombaHardwareInterface::get_name() const
{
  return "RoombaHardwareInterface";
}

hardware_interface::status RoombaHardwareInterface::get_status() const
{
  return hardware_interface::status::UNKNOWN;
}

}  // namespace roomba_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  roomba_hardware_interface::RoombaHardwareInterface, hardware_interface::SystemInterface)