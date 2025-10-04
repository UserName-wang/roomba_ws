#include "roomba_hardware_interface/roomba_hardware_driver.hpp"

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <errno.h>
#include <string.h>

namespace roomba_hardware_interface
{

RoombaHardware::RoombaHardware(const std::string & port, int baud_rate)
: fd_(-1), connected_(false), port_name_(port), baud_rate_(baud_rate)
{
}

RoombaHardware::~RoombaHardware()
{
  if (connected_) {
    // 根据用户要求，在退出时将Roomba设置为Stop模式
    stop();
    disconnect();
  }
  
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool RoombaHardware::connect()
{
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Attempting to connect to Roomba on %s", port_name_.c_str());
  
  // Open the port
  fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to open port %s: %s", port_name_.c_str(), strerror(errno));
    return false;
  }
  
  // Configure the serial port
  struct termios tty;
  
  // Get current attributes
  if (tcgetattr(fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Error getting terminal attributes: %s", strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }
  
  // Set baud rate
  speed_t baud_rate_const = get_baud_rate_constant(baud_rate_);
  if (baud_rate_const == B0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Unsupported baud rate: %d", baud_rate_);
    close(fd_);
    fd_ = -1;
    return false;
  }
  
  cfsetispeed(&tty, baud_rate_const);
  cfsetospeed(&tty, baud_rate_const);
  
  // Configure for raw mode
  cfmakeraw(&tty);
  
  // Set 8N1 (8 data bits, no parity, 1 stop bit)
  tty.c_cflag &= ~PARENB;  // No parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag &= ~CSIZE;   // Clear data bits
  tty.c_cflag |= CS8;      // 8 data bits
  
  // Apply settings
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Error setting terminal attributes: %s", strerror(errno));
    close(fd_);
    fd_ = -1;
    return false;
  }
  
  connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Successfully connected to Roomba on %s", port_name_.c_str());
  return true;
}

void RoombaHardware::disconnect()
{
  if (connected_) {
    // 根据用户要求，在断开连接时将Roomba设置为Stop模式
    stop();
    
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    
    connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Disconnected from Roomba");
  }
}

bool RoombaHardware::is_connected() const
{
  return connected_;
}

speed_t RoombaHardware::get_baud_rate_constant(int baud_rate)
{
  switch (baud_rate) {
    case 115200: return B115200;
    case 57600:  return B57600;
    case 38400:  return B38400;
    case 19200:  return B19200;
    case 9600:   return B9600;
    case 4800:   return B4800;
    case 2400:   return B2400;
    case 1200:   return B1200;
    case 600:    return B600;
    case 300:    return B300;
    default:     return B0;  // Invalid
  }
}

bool RoombaHardware::send_command(const std::vector<uint8_t> & command)
{
  if (!connected_ || fd_ < 0) {
    return false;
  }
  
  ssize_t bytes_written = write(fd_, command.data(), command.size());
  
  if (bytes_written != static_cast<ssize_t>(command.size())) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to send command: %s", strerror(errno));
    return false;
  }
  
  // Flush the output buffer
  tcdrain(fd_);
  
  // Small delay to ensure command is processed
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  return true;
}

bool RoombaHardware::send_command(const uint8_t * command, size_t length)
{
  if (!connected_ || fd_ < 0) {
    return false;
  }
  
  ssize_t bytes_written = write(fd_, command, length);
  
  if (bytes_written != static_cast<ssize_t>(length)) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to send command: %s", strerror(errno));
    return false;
  }
  
  // Flush the output buffer
  tcdrain(fd_);
  
  // Small delay to ensure command is processed
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  return true;
}

// Roomba control functions
void RoombaHardware::start()
{
  if (!connected_) return;
  uint8_t command = 128;  // START command
  send_command(&command, 1);
}

void RoombaHardware::reset()
{
  if (!connected_) return;
  uint8_t command = 7;  // RESET command
  send_command(&command, 1);
}

void RoombaHardware::stop()
{
  if (!connected_) return;
  uint8_t command = 173;  // STOP command
  send_command(&command, 1);
}

void RoombaHardware::safe_mode()
{
  if (!connected_) return;
  uint8_t command = 131;  // SAFE command
  send_command(&command, 1);
}

void RoombaHardware::full_mode()
{
  if (!connected_) return;
  uint8_t command = 132;  // FULL command
  send_command(&command, 1);
}

// Cleaning commands
void RoombaHardware::clean()
{
  if (!connected_) return;
  uint8_t command = 135;  // CLEAN command
  send_command(&command, 1);
}

void RoombaHardware::max_clean()
{
  if (!connected_) return;
  uint8_t command = 136;  // MAX command
  send_command(&command, 1);
}

void RoombaHardware::spot_clean()
{
  if (!connected_) return;
  uint8_t command = 134;  // SPOT command
  send_command(&command, 1);
}

void RoombaHardware::seek_dock()
{
  if (!connected_) return;
  uint8_t command = 143;  // SEEK DOCK command
  send_command(&command, 1);
}

void RoombaHardware::power_down()
{
  if (!connected_) return;
  uint8_t command = 133;  // POWER DOWN command
  send_command(&command, 1);
}

// Drive functions
void RoombaHardware::drive(int16_t velocity, int16_t radius)
{
  if (!connected_) return;
  
  uint8_t command[5];
  command[0] = 137;  // DRIVE command
  command[1] = (velocity >> 8) & 0xFF;  // Velocity high byte
  command[2] = velocity & 0xFF;         // Velocity low byte
  command[3] = (radius >> 8) & 0xFF;    // Radius high byte
  command[4] = radius & 0xFF;           // Radius low byte
  
  send_command(command, 5);
}

void RoombaHardware::drive_direct(int16_t right_velocity, int16_t left_velocity)
{
  if (!connected_) return;
  
  uint8_t command[5];
  command[0] = 145;  // DRIVE DIRECT command
  command[1] = (right_velocity >> 8) & 0xFF;  // Right velocity high byte
  command[2] = right_velocity & 0xFF;         // Right velocity low byte
  command[3] = (left_velocity >> 8) & 0xFF;   // Left velocity high byte
  command[4] = left_velocity & 0xFF;          // Left velocity low byte
  
  send_command(command, 5);
}

void RoombaHardware::drive_pwm(int16_t right_pwm, int16_t left_pwm)
{
  if (!connected_) return;
  
  uint8_t command[5];
  command[0] = 146;  // DRIVE PWM command
  command[1] = (right_pwm >> 8) & 0xFF;  // Right PWM high byte
  command[2] = right_pwm & 0xFF;         // Right PWM low byte
  command[3] = (left_pwm >> 8) & 0xFF;   // Left PWM high byte
  command[4] = left_pwm & 0xFF;          // Left PWM low byte
  
  send_command(command, 5);
}

// Motor control
void RoombaHardware::motors(uint8_t motor_bits)
{
  if (!connected_) return;
  
  uint8_t command[2];
  command[0] = 138;      // MOTORS command
  command[1] = motor_bits;
  
  send_command(command, 2);
}

void RoombaHardware::pwm_motors(int8_t main_brush_pwm, int8_t side_brush_pwm, int8_t vacuum_pwm)
{
  if (!connected_) return;
  
  uint8_t command[4];
  command[0] = 144;           // PWM MOTORS command
  command[1] = main_brush_pwm;
  command[2] = side_brush_pwm;
  command[3] = vacuum_pwm;
  
  send_command(command, 4);
}

// LED control
void RoombaHardware::leds(uint8_t led_bits, uint8_t power_color, uint8_t power_intensity)
{
  if (!connected_) return;
  
  uint8_t command[4];
  command[0] = 139;          // LEDS command
  command[1] = led_bits;
  command[2] = power_color;
  command[3] = power_intensity;
  
  send_command(command, 4);
}

// Sound
void RoombaHardware::play_song(uint8_t song_number)
{
  if (!connected_) return;
  
  uint8_t command[2];
  command[0] = 141;        // PLAY SONG command
  command[1] = song_number;
  
  send_command(command, 2);
}

void RoombaHardware::play_note(uint8_t note_number, uint8_t note_duration)
{
  if (!connected_) return;
  
  uint8_t command[3];
  command[0] = 140;         // PLAY NOTE command
  command[1] = note_number;
  command[2] = note_duration;
  
  send_command(command, 3);
}

void RoombaHardware::song(uint8_t song_number, const std::vector<uint8_t> & notes_and_durations)
{
  if (!connected_) return;
  
  if (notes_and_durations.size() % 2 != 0) {
    RCLCPP_WARN(rclcpp::get_logger("RoombaHardware"), "Song data should contain pairs of note and duration");
    return;
  }
  
  if (notes_and_durations.size() > 32) {
    RCLCPP_WARN(rclcpp::get_logger("RoombaHardware"), "Song data exceeds maximum of 16 notes");
    return;
  }
  
  std::vector<uint8_t> command;
  command.reserve(3 + notes_and_durations.size());
  
  command.push_back(140);  // SONG command
  command.push_back(song_number);
  command.push_back(static_cast<uint8_t>(notes_and_durations.size() / 2));
  command.insert(command.end(), notes_and_durations.begin(), notes_and_durations.end());
  
  send_command(command);
}

bool RoombaHardware::read_sensor_data(uint8_t packet_id, uint8_t * data, size_t length)
{
  if (!connected_ || fd_ < 0) {
    return false;
  }
  
  uint8_t command[2];
  command[0] = 142;      // SENSORS command
  command[1] = packet_id;
  
  if (!send_command(command, 2)) {
    return false;
  }
  
  // Small delay to allow sensor data to be prepared
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  
  // Read the response
  ssize_t bytes_read = read(fd_, data, length);
  if (bytes_read != static_cast<ssize_t>(length)) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to read sensor data: %s", strerror(errno));
    return false;
  }
  
  return true;
}

uint16_t RoombaHardware::read_sensor_uint16(uint8_t packet_id)
{
  uint8_t data[2];
  if (!read_sensor_data(packet_id, data, 2)) {
    return 0;
  }
  
  return (static_cast<uint16_t>(data[0]) << 8) | data[1];
}

uint8_t RoombaHardware::read_sensor_uint8(uint8_t packet_id)
{
  uint8_t data;
  if (!read_sensor_data(packet_id, &data, 1)) {
    return 0;
  }
  
  return data;
}

int16_t RoombaHardware::read_sensor_int16(uint8_t packet_id)
{
  uint8_t data[2];
  if (!read_sensor_data(packet_id, data, 2)) {
    return 0;
  }
  
  return (static_cast<int16_t>(data[0]) << 8) | data[1];
}

int8_t RoombaHardware::read_sensor_int8(uint8_t packet_id)
{
  uint8_t data;
  if (!read_sensor_data(packet_id, &data, 1)) {
    return 0;
  }
  
  return static_cast<int8_t>(data);
}

std::vector<uint8_t> RoombaHardware::read_sensor_packet(uint8_t packet_id, size_t length)
{
  std::vector<uint8_t> data(length);
  if (!read_sensor_data(packet_id, data.data(), length)) {
    data.clear();
  }
  
  return data;
}

// Sensor data functions
uint16_t RoombaHardware::read_battery_capacity()
{
  return read_sensor_uint16(26);  // Packet ID 26: Battery Capacity
}

uint16_t RoombaHardware::read_battery_charge()
{
  return read_sensor_uint16(25);  // Packet ID 25: Battery Charge
}

uint8_t RoombaHardware::read_charging_state()
{
  return read_sensor_uint8(21);   // Packet ID 21: Charging State
}

std::string RoombaHardware::get_charging_state_description()
{
  uint8_t state = read_charging_state();
  switch (state) {
    case 0: return "Not charging";
    case 1: return "Reconditioning Charging";
    case 2: return "Full Charging";
    case 3: return "Trickle Charging";
    case 4: return "Waiting";
    case 5: return "Charging Fault Condition";
    default: return "Unknown";
  }
}

int8_t RoombaHardware::read_temperature()
{
  return read_sensor_int8(24);    // Packet ID 24: Temperature
}

uint8_t RoombaHardware::read_battery_level()
{
  return read_sensor_uint8(25);   // Packet ID 25: Battery Charge (low byte)
}

bool RoombaHardware::read_bump_and_wheel_drop(uint8_t & bump_and_wheel_drop)
{
  bump_and_wheel_drop = read_sensor_uint8(7);  // Packet ID 7: Bumps and Wheel Drops
  return true;
}

bool RoombaHardware::read_cliff_sensors(
  bool & cliff_left, bool & cliff_front_left, 
  bool & cliff_front_right, bool & cliff_right)
{
  uint8_t data = read_sensor_uint8(10);  // Packet ID 10: Cliff Sensors
  if (data == 0) {
    return false;
  }
  
  cliff_left = data & 0x01;
  cliff_front_left = data & 0x02;
  cliff_front_right = data & 0x04;
  cliff_right = data & 0x08;
  
  return true;
}

bool RoombaHardware::read_buttons(bool & clean, bool & spot, bool & dock, bool & minute, bool & hour, bool & day, bool & schedule, bool & clock)
{
  uint8_t data = read_sensor_uint8(18);  // Packet ID 18: Buttons
  if (data == 0) {
    return false;
  }
  
  clean = data & 0x01;
  spot = data & 0x02;
  dock = data & 0x04;
  minute = data & 0x08;
  hour = data & 0x10;
  day = data & 0x20;
  schedule = data & 0x40;
  clock = data & 0x80;
  
  return true;
}

}  // namespace roomba_hardware_interface