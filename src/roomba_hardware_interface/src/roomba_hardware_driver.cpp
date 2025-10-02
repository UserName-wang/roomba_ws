#include "roomba_hardware_interface/roomba_hardware_driver.hpp"

#include <chrono>
#include <thread>
#include <vector>

namespace roomba_hardware_interface
{

RoombaHardware::RoombaHardware(const std::string & port, int baud_rate)
: port_(nullptr), connected_(false), port_name_(port), baud_rate_(baud_rate)
{
}

RoombaHardware::~RoombaHardware()
{
  if (connected_) {
    stop();
    disconnect();
  }
  
  if (port_) {
    sp_close(port_);
    sp_free_port(port_);
  }
}

bool RoombaHardware::connect()
{
  enum sp_return result;
  
  // Open the port
  result = sp_get_port_by_name(port_name_.c_str(), &port_);
  if (result != SP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to get port %s", port_name_.c_str());
    return false;
  }
  
  // Open the port
  result = sp_open(port_, SP_MODE_READ_WRITE);
  if (result != SP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to open port %s", port_name_.c_str());
    sp_free_port(port_);
    port_ = nullptr;
    return false;
  }
  
  // Set baud rate
  result = sp_set_baudrate(port_, baud_rate_);
  if (result != SP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to set baud rate");
    sp_close(port_);
    sp_free_port(port_);
    port_ = nullptr;
    return false;
  }
  
  // Set data bits, stop bits, and parity
  result = sp_set_bits(port_, 8);
  if (result != SP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to set data bits");
    sp_close(port_);
    return false;
  }
  
  result = sp_set_stopbits(port_, 1);
  if (result != SP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to set stop bits");
    sp_close(port_);
    return false;
  }
  
  result = sp_set_parity(port_, SP_PARITY_NONE);
  if (result != SP_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to set parity");
    sp_close(port_);
    return false;
  }
  
  connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Connected to Roomba on %s", port_name_.c_str());
  return true;
}

void RoombaHardware::disconnect()
{
  if (connected_) {
    stop();
    sp_close(port_);
    connected_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Disconnected from Roomba");
  }
}

bool RoombaHardware::is_connected() const
{
  return connected_;
}

void RoombaHardware::start()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {START_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Started Roomba OI");
}

void RoombaHardware::reset()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {RESET_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Reset Roomba");
}

void RoombaHardware::stop()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {STOP_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Stopped Roomba OI");
}

void RoombaHardware::safe_mode()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {SAFE_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Set Roomba to Safe Mode");
}

void RoombaHardware::full_mode()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {FULL_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Set Roomba to Full Mode");
}

void RoombaHardware::clean()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {CLEAN_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Start cleaning");
}

void RoombaHardware::max_clean()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {MAX_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Start max cleaning");
}

void RoombaHardware::spot_clean()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {SPOT_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Start spot cleaning");
}

void RoombaHardware::seek_dock()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {SEEK_DOCK_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Seek dock");
}

void RoombaHardware::power_down()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {POWER_CMD};
  send_command(cmd);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Power down");
}

void RoombaHardware::drive(int16_t velocity, int16_t radius)
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {
    DRIVE_CMD,
    static_cast<uint8_t>((velocity >> 8) & 0xFF),   // velocity high byte
    static_cast<uint8_t>(velocity & 0xFF),          // velocity low byte
    static_cast<uint8_t>((radius >> 8) & 0xFF),     // radius high byte
    static_cast<uint8_t>(radius & 0xFF)             // radius low byte
  };
  send_command(cmd);
}

void RoombaHardware::drive_direct(int16_t right_velocity, int16_t left_velocity)
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {
    DRIVE_DIRECT_CMD,
    static_cast<uint8_t>((right_velocity >> 8) & 0xFF),  // right high byte
    static_cast<uint8_t>(right_velocity & 0xFF),         // right low byte
    static_cast<uint8_t>((left_velocity >> 8) & 0xFF),   // left high byte
    static_cast<uint8_t>(left_velocity & 0xFF)           // left low byte
  };
  send_command(cmd);
}

void RoombaHardware::drive_pwm(int16_t right_pwm, int16_t left_pwm)
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {
    DRIVE_PWM_CMD,
    static_cast<uint8_t>((right_pwm >> 8) & 0xFF),  // right high byte
    static_cast<uint8_t>(right_pwm & 0xFF),         // right low byte
    static_cast<uint8_t>((left_pwm >> 8) & 0xFF),   // left high byte
    static_cast<uint8_t>(left_pwm & 0xFF)           // left low byte
  };
  send_command(cmd);
}

void RoombaHardware::motors(uint8_t motor_bits)
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {MOTORS_CMD, motor_bits};
  send_command(cmd);
}

void RoombaHardware::pwm_motors(int8_t main_brush_pwm, int8_t side_brush_pwm, int8_t vacuum_pwm)
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {
    PWM_MOTORS_CMD,
    static_cast<uint8_t>(main_brush_pwm),
    static_cast<uint8_t>(side_brush_pwm),
    static_cast<uint8_t>(vacuum_pwm)
  };
  send_command(cmd);
}

void RoombaHardware::leds(uint8_t led_bits, uint8_t power_color, uint8_t power_intensity)
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {LEDS_CMD, led_bits, power_color, power_intensity};
  send_command(cmd);
}

void RoombaHardware::song(uint8_t song_number, const std::vector<std::pair<uint8_t, uint8_t>>& notes)
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {SONG_CMD, song_number, static_cast<uint8_t>(notes.size())};
  
  for (const auto& note : notes) {
    cmd.push_back(note.first);  // Note number
    cmd.push_back(note.second); // Duration
  }
  
  send_command(cmd);
}

void RoombaHardware::play(uint8_t song_number)
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {PLAY_CMD, song_number};
  send_command(cmd);
}

uint8_t RoombaHardware::read_charging_state()
{
  return read_single_byte_sensor(CHARGING_STATE_PKT);
}

std::string RoombaHardware::get_charging_state_description()
{
  uint8_t state = read_charging_state();
  switch (state) {
    case NOT_CHARGING:
      return "Not charging";
    case RECONDITIONING_CHARGING:
      return "Reconditioning charging";
    case FULL_CHARGING:
      return "Full charging";
    case TRICKLE_CHARGING:
      return "Trickle charging";
    case WAITING:
      return "Waiting";
    case CHARGING_FAULT_CONDITION:
      return "Charging fault condition";
    default:
      return "Unknown state";
  }
}

bool RoombaHardware::has_fault()
{
  uint8_t state = read_charging_state();
  return (state == CHARGING_FAULT_CONDITION);
}

void RoombaHardware::clear_fault()
{
  if (!connected_) return;
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Attempting to clear fault by resetting Roomba...");
  
  // Send reset command
  std::vector<uint8_t> cmd = {RESET_CMD};
  send_command(cmd);
  
  // Wait for reset to complete
  std::this_thread::sleep_for(std::chrono::seconds(3));
  
  // Restart OI
  start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  RCLCPP_INFO(rclcpp::get_logger("RoombaHardware"), "Roomba reset completed");
}

uint8_t RoombaHardware::read_bumps_and_wheel_drops()
{
  return read_single_byte_sensor(BUMPS_AND_WHEEL_DROPS_PKT);
}

bool RoombaHardware::is_left_bump()
{
  uint8_t sensor_data = read_bumps_and_wheel_drops();
  return (sensor_data & 0x02) != 0;
}

bool RoombaHardware::is_right_bump()
{
  uint8_t sensor_data = read_bumps_and_wheel_drops();
  return (sensor_data & 0x01) != 0;
}

bool RoombaHardware::is_left_wheel_drop()
{
  uint8_t sensor_data = read_bumps_and_wheel_drops();
  return (sensor_data & 0x08) != 0;
}

bool RoombaHardware::is_right_wheel_drop()
{
  uint8_t sensor_data = read_bumps_and_wheel_drops();
  return (sensor_data & 0x04) != 0;
}

uint8_t RoombaHardware::read_wall_sensor()
{
  return read_single_byte_sensor(WALL_PKT);
}

uint8_t RoombaHardware::read_cliff_left()
{
  return read_single_byte_sensor(CLIFF_LEFT_PKT);
}

uint8_t RoombaHardware::read_cliff_front_left()
{
  return read_single_byte_sensor(CLIFF_FRONT_LEFT_PKT);
}

uint8_t RoombaHardware::read_cliff_front_right()
{
  return read_single_byte_sensor(CLIFF_FRONT_RIGHT_PKT);
}

uint8_t RoombaHardware::read_cliff_right()
{
  return read_single_byte_sensor(CLIFF_RIGHT_PKT);
}

uint8_t RoombaHardware::read_virtual_wall()
{
  return read_single_byte_sensor(VIRTUAL_WALL_PKT);
}

uint8_t RoombaHardware::read_wheel_overcurrents()
{
  return read_single_byte_sensor(WHEEL_OVERCURRENTS_PKT);
}

bool RoombaHardware::is_left_wheel_overcurrent()
{
  uint8_t sensor_data = read_wheel_overcurrents();
  return (sensor_data & 0x10) != 0;
}

bool RoombaHardware::is_right_wheel_overcurrent()
{
  uint8_t sensor_data = read_wheel_overcurrents();
  return (sensor_data & 0x08) != 0;
}

bool RoombaHardware::is_main_brush_overcurrent()
{
  uint8_t sensor_data = read_wheel_overcurrents();
  return (sensor_data & 0x04) != 0;
}

bool RoombaHardware::is_side_brush_overcurrent()
{
  uint8_t sensor_data = read_wheel_overcurrents();
  return (sensor_data & 0x01) != 0;
}

int16_t RoombaHardware::read_distance()
{
  return read_two_byte_signed_sensor(DISTANCE_PKT);
}

int16_t RoombaHardware::read_angle()
{
  return read_two_byte_signed_sensor(ANGLE_PKT);
}

uint16_t RoombaHardware::read_voltage()
{
  return read_two_byte_sensor(VOLTAGE_PKT);
}

int16_t RoombaHardware::read_current()
{
  return read_two_byte_signed_sensor(CURRENT_PKT);
}

int8_t RoombaHardware::read_temperature()
{
  uint8_t temp = read_single_byte_sensor(TEMPERATURE_PKT);
  return static_cast<int8_t>(temp);
}

uint16_t RoombaHardware::read_battery_charge()
{
  return read_two_byte_sensor(BATTERY_CHARGE_PKT);
}

uint16_t RoombaHardware::read_battery_capacity()
{
  return read_two_byte_sensor(BATTERY_CAPACITY_PKT);
}

uint8_t RoombaHardware::readOIMode()
{
  return read_single_byte_sensor(OI_MODE_PKT);
}

bool RoombaHardware::readSongPlaying()
{
  return read_single_byte_sensor(SONG_PLAYING_PKT) != 0;
}

void RoombaHardware::stream_enable(const std::vector<uint8_t>& packet_ids)
{
  if (!connected_ || packet_ids.empty()) return;
  
  std::vector<uint8_t> cmd = {STREAM_CMD, static_cast<uint8_t>(packet_ids.size())};
  cmd.insert(cmd.end(), packet_ids.begin(), packet_ids.end());
  send_command(cmd);
}

void RoombaHardware::stream_disable()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {STREAM_CMD, 0};
  send_command(cmd);
}

void RoombaHardware::stream_pause()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {PAUSE_RESUME_STREAM_CMD, 0};
  send_command(cmd);
}

void RoombaHardware::stream_resume()
{
  if (!connected_) return;
  
  std::vector<uint8_t> cmd = {PAUSE_RESUME_STREAM_CMD, 1};
  send_command(cmd);
}

bool RoombaHardware::send_command(const std::vector<uint8_t>& command)
{
  if (!connected_) return false;
  
  enum sp_return result;
  result = sp_blocking_write(port_, command.data(), command.size(), 1000);
  
  if (result < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to send command to Roomba");
    connected_ = false;
    return false;
  }
  
  return true;
}

std::vector<uint8_t> RoombaHardware::read_sensor_data(uint8_t packet_id, size_t data_length)
{
  if (!connected_) return {};
  
  std::vector<uint8_t> cmd = {QUERY_LIST_CMD, 1, packet_id};
  if (!send_command(cmd)) {
    return {};
  }
  
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  
  std::vector<uint8_t> result(data_length);
  int bytes_read = sp_blocking_read(port_, result.data(), data_length, 1000);
  
  if (bytes_read < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Failed to read sensor data from Roomba");
    connected_ = false;
    return {};
  }
  
  // 确保读取到完整的数据
  if (static_cast<size_t>(bytes_read) != data_length) {
    RCLCPP_ERROR(rclcpp::get_logger("RoombaHardware"), "Incomplete sensor data read: expected %zu bytes, got %d", data_length, bytes_read);
    return {};
  }
  
  return result;
}

uint8_t RoombaHardware::read_single_byte_sensor(uint8_t packet_id)
{
  auto data = read_sensor_data(packet_id, 1);
  if (!data.empty()) {
    return data[0];
  }
  return 0;
}

uint16_t RoombaHardware::read_two_byte_sensor(uint8_t packet_id)
{
  auto data = read_sensor_data(packet_id, 2);
  if (data.size() >= 2) {
    return (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
  }
  return 0;
}

int16_t RoombaHardware::read_two_byte_signed_sensor(uint8_t packet_id)
{
  auto data = read_sensor_data(packet_id, 2);
  if (data.size() >= 2) {
    return (static_cast<int16_t>(data[0]) << 8) | static_cast<int16_t>(data[1]);
  }
  return 0;
}

}  // namespace roomba_hardware_interface