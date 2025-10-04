#include "rclcpp/rclcpp.hpp"
#include "roomba_hardware_interface/roomba_hardware_driver.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class RoombaDriverTest : public rclcpp::Node
{
public:
  RoombaDriverTest() : Node("roomba_driver_test")
  {
    // Declare parameters
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud_rate", 115200);
    
    // Get parameters
    std::string port;
    int baud_rate;
    this->get_parameter("port", port);
    this->get_parameter("baud_rate", baud_rate);
    
    // Create Roomba hardware instance
    roomba_ = std::make_unique<roomba_hardware_interface::RoombaHardware>(port, baud_rate);
    
    // Create subscriber for cmd_vel
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&RoombaDriverTest::cmdVelCallback, this, std::placeholders::_1));
    
    // Create publisher for status messages
    status_pub_ = this->create_publisher<std_msgs::msg::String>("roomba_status", 10);
    
    // Timer for periodic status updates
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&RoombaDriverTest::timerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), "Roomba Driver Test node initialized");
    RCLCPP_INFO(this->get_logger(), "Port: %s, Baud rate: %d", port.c_str(), baud_rate);
  }
  
  bool connect()
  {
    RCLCPP_INFO(this->get_logger(), "Attempting to connect to Roomba...");
    if (!roomba_->connect()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to Roomba");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully connected to Roomba!");
    
    // Start and set to safe mode
    roomba_->start();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    roomba_->safe_mode();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    RCLCPP_INFO(this->get_logger(), "Roomba set to Safe Mode");
    return true;
  }
  
  void disconnect()
  {
    if (roomba_) {
      roomba_->stop();
      roomba_->disconnect();
      RCLCPP_INFO(this->get_logger(), "Disconnected from Roomba");
    }
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!roomba_) {
      RCLCPP_WARN(this->get_logger(), "Roomba not connected");
      return;
    }
    
    // Convert Twist to wheel velocities
    // Parameters for Roomba
    const double wheel_base = 0.235;  // meters
    const double wheel_radius = 0.03; // meters
    
    // Calculate wheel velocities
    double left_velocity = (msg->linear.x - msg->angular.z * wheel_base / 2.0) / wheel_radius;
    double right_velocity = (msg->linear.x + msg->angular.z * wheel_base / 2.0) / wheel_radius;
    
    // Convert to mm/s and limit to Roomba's capabilities (-500 to 500 mm/s)
    int16_t left_mm_s = std::max(static_cast<int16_t>(-500), 
                                 std::min(static_cast<int16_t>(500), 
                                          static_cast<int16_t>(left_velocity * 1000)));
    int16_t right_mm_s = std::max(static_cast<int16_t>(-500), 
                                  std::min(static_cast<int16_t>(500), 
                                           static_cast<int16_t>(right_velocity * 1000)));
    
    // Send command to Roomba
    roomba_->drive_direct(right_mm_s, left_mm_s);
    
    RCLCPP_DEBUG(this->get_logger(), "Sent cmd_vel: linear.x=%.2f, angular.z=%.2f", 
                 msg->linear.x, msg->angular.z);
    RCLCPP_DEBUG(this->get_logger(), "Wheel velocities: left=%d mm/s, right=%d mm/s", 
                 left_mm_s, right_mm_s);
  }
  
  void timerCallback()
  {
    if (!roomba_) {
      return;
    }
    
    // Read sensor data periodically
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    status_msg->data = "Charging state: " + roomba_->get_charging_state_description() + 
                      ", Battery: " + std::to_string(roomba_->read_battery_charge()) + " mAh / " + 
                      std::to_string(roomba_->read_battery_capacity()) + " mAh";
    
    status_pub_->publish(std::move(status_msg));
    
    RCLCPP_DEBUG(this->get_logger(), "Published status: %s", status_msg->data.c_str());
  }
  
  std::unique_ptr<roomba_hardware_interface::RoombaHardware> roomba_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<RoombaDriverTest>();
  
  if (!node->connect()) {
    rclcpp::shutdown();
    return -1;
  }
  
  rclcpp::spin(node);
  
  node->disconnect();
  rclcpp::shutdown();
  
  return 0;
}