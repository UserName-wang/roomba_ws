#include "roomba_hardware_interface/roomba_hardware_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

int main(int argc, char *argv[])
{
  std::string port = "/dev/ttyUSB0";
  if (argc > 1) {
    port = argv[1];
  }
  
  std::cout << "Roomba Hardware Interface Example" << std::endl;
  std::cout << "Using port: " << port << std::endl;
  
  // Create Roomba instance
  auto roomba = std::make_unique<roomba_hardware_interface::RoombaHardware>(port);
  
  // Connect to Roomba
  std::cout << "Connecting to Roomba..." << std::endl;
  if (!roomba->connect()) {
    std::cerr << "Failed to connect to Roomba!" << std::endl;
    return -1;
  }
  
  std::cout << "Connected successfully!" << std::endl;
  
  // Start and set to safe mode
  roomba->start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  roomba->safe_mode();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Check initial state
  std::cout << "Initial charging state: " << roomba->get_charging_state_description() << std::endl;
  
  // Demonstrate sensor reading
  std::cout << "\n=== Sensor Readings ===" << std::endl;
  uint8_t bump_data;
  if (roomba->read_bump_and_wheel_drop(bump_data)) {
    std::cout << "Bump sensors: " << (bump_data ? "YES" : "NO") << std::endl;
  }
  
  bool cliff_left, cliff_front_left, cliff_front_right, cliff_right;
  if (roomba->read_cliff_sensors(cliff_left, cliff_front_left, cliff_front_right, cliff_right)) {
    std::cout << "Cliff left: " << (cliff_left ? "YES" : "NO") << std::endl;
    std::cout << "Cliff front left: " << (cliff_front_left ? "YES" : "NO") << std::endl;
    std::cout << "Cliff front right: " << (cliff_front_right ? "YES" : "NO") << std::endl;
    std::cout << "Cliff right: " << (cliff_right ? "YES" : "NO") << std::endl;
  }
  
  std::cout << "Battery temperature: " << static_cast<int>(roomba->read_temperature()) << " C" << std::endl;
  std::cout << "Battery charge: " << roomba->read_battery_charge() << " mAh" << std::endl;
  std::cout << "Battery capacity: " << roomba->read_battery_capacity() << " mAh" << std::endl;
  
  // Demonstrate motor control
  std::cout << "\n=== Motor Control Demo ===" << std::endl;
  std::cout << "Turning on main brush..." << std::endl;
  roomba->motors(1 << 2); // Main brush
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Turning on side brush..." << std::endl;
  roomba->motors(1 << 1); // Side brush
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Turning on vacuum..." << std::endl;
  roomba->motors(1 << 0); // Vacuum
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Turning off all motors..." << std::endl;
  roomba->motors(0);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  // Demonstrate movement
  std::cout << "\n=== Movement Demo ===" << std::endl;
  std::cout << "Driving forward..." << std::endl;
  roomba->drive_direct(100, 100); // Move forward
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Turning left..." << std::endl;
  roomba->drive_direct(-100, 100); // Turn left
  std::this_thread::sleep_for(std::chrono::seconds(2));
  
  std::cout << "Stopping..." << std::endl;
  roomba->drive_direct(0, 0); // Stop
  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  // Demonstrate LEDs
  std::cout << "\n=== LED Demo ===" << std::endl;
  roomba->leds(0xF, 128, 255); // Turn on all LEDs
  std::this_thread::sleep_for(std::chrono::seconds(2));
  roomba->leds(0, 0, 0); // Turn off all LEDs
  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  // Demonstrate sound
  std::cout << "\n=== Sound Demo ===" << std::endl;
  std::vector<uint8_t> notes_and_durations = {
    60, 16, // C note, 16/64th notes duration
    64, 16, // E note
    67, 16, // G note
    72, 32  // C note, 32/64th notes duration
  };
  roomba->song(0, notes_and_durations);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  roomba->play_song(0);
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  // Final state check
  std::cout << "\nFinal charging state: " << roomba->get_charging_state_description() << std::endl;
  
  // Disconnect
  roomba->stop();
  roomba->disconnect();
  
  std::cout << "Disconnected. Demo complete!" << std::endl;
  
  return 0;
}