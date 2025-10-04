#ifndef ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_HPP_
#define ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

// Use system serial library
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace roomba_hardware_interface
{

class RoombaHardware
{
public:
  explicit RoombaHardware(const std::string & port = "/dev/ttyUSB0", int baud_rate = 115200);
  ~RoombaHardware();

  // Connection functions
  bool connect();
  void disconnect();
  bool is_connected() const;

  // Roomba control functions
  void start();
  void reset();
  void stop();
  void safe_mode();
  void full_mode();
  
  // Cleaning commands
  void clean();
  void max_clean();
  void spot_clean();
  void seek_dock();
  void power_down();
  
  // Drive functions
  void drive(int16_t velocity, int16_t radius);
  void drive_direct(int16_t right_velocity, int16_t left_velocity);
  void drive_pwm(int16_t right_pwm, int16_t left_pwm);
  
  // Motor control
  void motors(uint8_t motor_bits);
  void pwm_motors(int8_t main_brush_pwm, int8_t side_brush_pwm, int8_t vacuum_pwm);
  
  // LED control
  void leds(uint8_t led_bits, uint8_t power_color, uint8_t power_intensity);
  
  // Sound
  void play_song(uint8_t song_number);
  void play_note(uint8_t note_number, uint8_t note_duration);
  void song(uint8_t song_number, const std::vector<uint8_t> & notes_and_durations);
  
  // Sensors
  uint16_t read_sensor_uint16(uint8_t packet_id);
  uint8_t read_sensor_uint8(uint8_t packet_id);
  int16_t read_sensor_int16(uint8_t packet_id);
  int8_t read_sensor_int8(uint8_t packet_id);
  std::vector<uint8_t> read_sensor_packet(uint8_t packet_id, size_t length);
  
  // Sensor data
  uint16_t read_battery_capacity();
  uint16_t read_battery_charge();
  uint8_t read_charging_state();
  std::string get_charging_state_description();
  int8_t read_temperature();
  uint8_t read_battery_level();
  
  // Bumps and wheeldrops
  bool read_bump_and_wheel_drop(uint8_t & bump_and_wheel_drop);
  
  // Cliff sensors
  bool read_cliff_sensors(
    bool & cliff_left, bool & cliff_front_left, 
    bool & cliff_front_right, bool & cliff_right);
  
  // Buttons
  bool read_buttons(bool & clean, bool & spot, bool & dock, bool & minute, bool & hour, bool & day, bool & schedule, bool & clock);

private:
  int fd_;  // File descriptor for the serial port
  bool connected_;
  std::string port_name_;
  int baud_rate_;
  
  bool send_command(const std::vector<uint8_t> & command);
  bool send_command(const uint8_t * command, size_t length);
  bool read_sensor_data(uint8_t packet_id, uint8_t * data, size_t length);
  speed_t get_baud_rate_constant(int baud_rate);
};

}  // namespace roomba_hardware_interface

#endif  // ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_HPP_