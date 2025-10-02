#include "roomba_hardware_interface/roomba_hardware_driver.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  std::cout << "Roomba Hardware Interface Example" << std::endl;
  
  // Create Roomba hardware instance
  auto roomba = std::make_unique<roomba_hardware_interface::RoombaHardware>("/dev/ttyUSB0", 115200);
  
  // Connect to Roomba
  std::cout << "Attempting to connect to Roomba..." << std::endl;
  if (!roomba->connect()) {
    std::cerr << "Failed to connect to Roomba. Please check the connection and try again." << std::endl;
    return -1;
  }
  
  std::cout << "Successfully connected to Roomba!" << std::endl;
  
  // Start and set to safe mode
  std::cout << "Starting Roomba..." << std::endl;
  roomba->start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  std::cout << "Setting Roomba to Safe Mode..." << std::endl;
  roomba->safe_mode();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Check initial state
  std::cout << "Initial charging state: " << roomba->get_charging_state_description() << std::endl;
  
  if (roomba->has_fault()) {
    std::cout << "Fault detected! Attempting to clear..." << std::endl;
    roomba->clear_fault();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Fault cleared. Current state: " << roomba->get_charging_state_description() << std::endl;
  }
  
  // Demonstrate sensor reading
  std::cout << "\n=== Sensor Readings ===" << std::endl;
  std::cout << "Left bump: " << (roomba->is_left_bump() ? "YES" : "NO") << std::endl;
  std::cout << "Right bump: " << (roomba->is_right_bump() ? "YES" : "NO") << std::endl;
  std::cout << "Wall sensor: " << (roomba->read_wall_sensor() ? "YES" : "NO") << std::endl;
  std::cout << "Cliff left: " << (roomba->read_cliff_left() ? "YES" : "NO") << std::endl;
  std::cout << "Cliff front left: " << (roomba->read_cliff_front_left() ? "YES" : "NO") << std::endl;
  std::cout << "Cliff front right: " << (roomba->read_cliff_front_right() ? "YES" : "NO") << std::endl;
  std::cout << "Cliff right: " << (roomba->read_cliff_right() ? "YES" : "NO") << std::endl;
  std::cout << "Virtual wall: " << (roomba->read_virtual_wall() ? "YES" : "NO") << std::endl;
  std::cout << "Distance traveled: " << roomba->read_distance() << " mm" << std::endl;
  std::cout << "Angle turned: " << roomba->read_angle() << " degrees" << std::endl;
  std::cout << "Battery voltage: " << roomba->read_voltage() << " mV" << std::endl;
  std::cout << "Battery current: " << roomba->read_current() << " mA" << std::endl;
  std::cout << "Battery temperature: " << static_cast<int>(roomba->read_temperature()) << " C" << std::endl;
  std::cout << "Battery charge: " << roomba->read_battery_charge() << " mAh" << std::endl;
  std::cout << "Battery capacity: " << roomba->read_battery_capacity() << " mAh" << std::endl;
  
  // Demonstrate motor control
  std::cout << "\n=== Motor Control Demo ===" << std::endl;
  std::cout << "Turning on main brush..." << std::endl;
  roomba->motors(0x04); // Main brush
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Turning on side brush..." << std::endl;
  roomba->motors(0x01); // Side brush
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Turning on vacuum..." << std::endl;
  roomba->motors(0x02); // Vacuum
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Turning off all motors..." << std::endl;
  roomba->motors(0x00);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  // Demonstrate movement
  std::cout << "\n=== Movement Demo ===" << std::endl;
  std::cout << "Moving forward..." << std::endl;
  roomba->drive_direct(100, 100); // Straight forward
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Turning left..." << std::endl;
  roomba->drive_direct(50, -50); // Turn left
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Stopping..." << std::endl;
  roomba->drive_direct(0, 0); // Stop
  
  // Demonstrate sound
  std::cout << "\n=== Sound Demo ===" << std::endl;
  std::vector<std::pair<uint8_t, uint8_t>> notes = {
    {60, 16}, // C note, 16/64th notes duration
    {64, 16}, // E note
    {67, 16}, // G note
    {72, 32}  // C note, 32/64th notes duration
  };
  roomba->song(0, notes);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  roomba->play(0);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  // Final state check
  std::cout << "\nFinal charging state: " << roomba->get_charging_state_description() << std::endl;
  
  // Disconnect
  std::cout << "Disconnecting from Roomba..." << std::endl;
  roomba->disconnect();
  
  std::cout << "Example completed successfully!" << std::endl;
  
  // Shutdown ROS 2
  rclcpp::shutdown();
  
  return 0;
}