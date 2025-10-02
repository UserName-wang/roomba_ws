#ifndef ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_HPP_
#define ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"

// Use system serial library
extern "C" {
#include <libserialport.h>
}

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
  void song(uint8_t song_number, const std::vector<std::pair<uint8_t, uint8_t>>& notes);
  void play(uint8_t song_number);
  
  // Sensor functions
  uint8_t read_charging_state();
  std::string get_charging_state_description();
  bool has_fault();
  void clear_fault();
  
  // Sensor reading functions
  uint8_t read_bumps_and_wheel_drops();
  bool is_left_bump();
  bool is_right_bump();
  bool is_left_wheel_drop();
  bool is_right_wheel_drop();
  
  uint8_t read_wall_sensor();
  uint8_t read_cliff_left();
  uint8_t read_cliff_front_left();
  uint8_t read_cliff_front_right();
  uint8_t read_cliff_right();
  uint8_t read_virtual_wall();
  
  uint8_t read_wheel_overcurrents();
  bool is_left_wheel_overcurrent();
  bool is_right_wheel_overcurrent();
  bool is_main_brush_overcurrent();
  bool is_side_brush_overcurrent();
  
  int16_t read_distance();
  int16_t read_angle();
  uint16_t read_voltage();
  int16_t read_current();
  int8_t read_temperature();
  uint16_t read_battery_charge();
  uint16_t read_battery_capacity();
  
  uint8_t readOIMode();
  bool readSongPlaying();
  
  // Streaming
  void stream_enable(const std::vector<uint8_t>& packet_ids);
  void stream_disable();
  void stream_pause();
  void stream_resume();

private:
  // Serial communication
  struct sp_port *port_;
  bool connected_;
  
  // Connection parameters
  std::string port_name_;
  int baud_rate_;
  
  // Roomba OI commands
  enum Command {
    START_CMD = 128,
    RESET_CMD = 7,
    STOP_CMD = 173,
    BAUD_CMD = 129,
    CONTROL_CMD = 130,
    SAFE_CMD = 131,
    FULL_CMD = 132,
    POWER_CMD = 133,
    SPOT_CMD = 134,
    CLEAN_CMD = 135,
    MAX_CMD = 136,
    DRIVE_CMD = 137,
    MOTORS_CMD = 138,
    LEDS_CMD = 139,
    SONG_CMD = 140,
    PLAY_CMD = 141,
    SEEK_DOCK_CMD = 143,
    PWM_MOTORS_CMD = 144,
    DRIVE_DIRECT_CMD = 145,
    DRIVE_PWM_CMD = 146,
    STREAM_CMD = 148,
    QUERY_LIST_CMD = 149,
    PAUSE_RESUME_STREAM_CMD = 150,
    SCHEDULING_LEDS_CMD = 162,
    DIGIT_LEDS_RAW_CMD = 163,
    DIGIT_LEDS_ASCII_CMD = 164,
    BUTTONS_CMD = 165,
    SCHEDULE_CMD = 167,
    SET_DAY_TIME_CMD = 168
  };

  // Sensor packet IDs
  enum SensorPacketID {
    BUMPS_AND_WHEEL_DROPS_PKT = 7,
    WALL_PKT = 8,
    CLIFF_LEFT_PKT = 9,
    CLIFF_FRONT_LEFT_PKT = 10,
    CLIFF_FRONT_RIGHT_PKT = 11,
    CLIFF_RIGHT_PKT = 12,
    VIRTUAL_WALL_PKT = 13,
    WHEEL_OVERCURRENTS_PKT = 14,
    DIRT_DETECT_PKT = 15,
    UNUSED_BYTE_PKT = 16,
    INFRA_RED_OMNI_PKT = 17,
    BUTTONS_PKT = 18,
    DISTANCE_PKT = 19,
    ANGLE_PKT = 20,
    CHARGING_STATE_PKT = 21,
    VOLTAGE_PKT = 22,
    CURRENT_PKT = 23,
    TEMPERATURE_PKT = 24,
    BATTERY_CHARGE_PKT = 25,
    BATTERY_CAPACITY_PKT = 26,
    WALL_SIGNAL_PKT = 27,
    CLIFF_LEFT_SIGNAL_PKT = 28,
    CLIFF_FRONT_LEFT_SIGNAL_PKT = 29,
    CLIFF_FRONT_RIGHT_SIGNAL_PKT = 30,
    CLIFF_RIGHT_SIGNAL_PKT = 31,
    CHARGING_SOURCES_AVAILABLE_PKT = 34,
    OI_MODE_PKT = 35,
    SONG_NUMBER_PKT = 36,
    SONG_PLAYING_PKT = 37,
    NUMBER_OF_STREAM_PACKETS_PKT = 38,
    REQUESTED_VELOCITY_PKT = 39,
    REQUESTED_RADIUS_PKT = 40,
    REQUESTED_RIGHT_VELOCITY_PKT = 41,
    REQUESTED_LEFT_VELOCITY_PKT = 42
  };

  // Charging states
  enum ChargingState {
    NOT_CHARGING = 0,
    RECONDITIONING_CHARGING = 1,
    FULL_CHARGING = 2,
    TRICKLE_CHARGING = 3,
    WAITING = 4,
    CHARGING_FAULT_CONDITION = 5
  };

  // OI Modes
  enum OIMode {
    OFF_MODE = 0,
    PASSIVE_MODE = 1,
    SAFE_MODE = 2,
    FULL_MODE = 3
  };

  // Helper functions
  bool send_command(const std::vector<uint8_t>& command);
  std::vector<uint8_t> read_sensor_data(uint8_t packet_id, size_t data_length);
  uint8_t read_single_byte_sensor(uint8_t packet_id);
  uint16_t read_two_byte_sensor(uint8_t packet_id);
  int16_t read_two_byte_signed_sensor(uint8_t packet_id);
};

}  // namespace roomba_hardware_interface

#endif  // ROOMBA_HARDWARE_INTERFACE__ROOMBA_HARDWARE_HPP_