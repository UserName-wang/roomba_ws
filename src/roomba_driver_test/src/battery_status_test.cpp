#include <iostream>
#include <chrono>
#include <thread>
#include <signal.h>
#include "roomba_hardware_interface/roomba_hardware_driver.hpp"

using namespace roomba_hardware_interface;

RoombaHardware* roomba = nullptr;

void signal_handler(int signal)
{
  std::cout << "\nReceived signal " << signal << ", stopping Roomba..." << std::endl;
  
  if (roomba) {
    roomba->stop();
    delete roomba;
  }
  
  exit(0);
}

int main(int argc, char** argv)
{
  // 注册信号处理函数，以便在程序退出时发送STOP命令
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  
  std::cout << "Roomba Battery Status Test" << std::endl;
  std::cout << "=========================" << std::endl;
  
  // 创建Roomba硬件驱动实例
  roomba = new RoombaHardware("/dev/ttyUSB0", 115200);
  
  // 连接到Roomba
  if (!roomba->connect()) {
    std::cerr << "Failed to connect to Roomba!" << std::endl;
    delete roomba;
    return 1;
  }
  
  std::cout << "Successfully connected to Roomba!" << std::endl;
  
  // 进入被动模式（Passive Mode）
  roomba->start();
  std::cout << "Roomba set to Passive Mode" << std::endl;
  
  // 循环读取并显示电池状态
  while (true) {
    try {
      // 读取电池信息
      uint16_t battery_charge = roomba->read_battery_charge();
      uint16_t battery_capacity = roomba->read_battery_capacity();
      int8_t temperature = roomba->read_temperature();
      uint8_t charging_state = roomba->read_charging_state();
      std::string charging_state_desc = roomba->get_charging_state_description();
      uint8_t battery_level = roomba->read_battery_level();
      
      // 清屏并显示电池信息
      std::cout << "\033[2J\033[1;1H"; // 清屏并移动光标到左上角
      std::cout << "Roomba Battery Status (Press Ctrl+C to exit)" << std::endl;
      std::cout << "===========================================" << std::endl;
      std::cout << "Battery Charge:     " << battery_charge << " mAh" << std::endl;
      std::cout << "Battery Capacity:   " << battery_capacity << " mAh" << std::endl;
      std::cout << "Battery Percentage: " << (battery_capacity > 0 ? (battery_charge * 100 / battery_capacity) : 0) << "%" << std::endl;
      std::cout << "Battery Level:      " << (int)battery_level << std::endl;
      std::cout << "Temperature:        " << (int)temperature << " °C" << std::endl;
      std::cout << "Charging State:     " << (int)charging_state << " (" << charging_state_desc << ")" << std::endl;
      
      // 等待2秒
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    catch (const std::exception& e) {
      std::cerr << "Error reading sensor data: " << e.what() << std::endl;
      break;
    }
  }
  
  // 正常退出时发送STOP命令
  std::cout << "Stopping Roomba..." << std::endl;
  roomba->stop();
  delete roomba;
  
  return 0;
}