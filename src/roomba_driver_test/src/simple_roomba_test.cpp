#include "roomba_hardware_interface/roomba_hardware_driver.hpp"
#include <iostream>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  std::string port = "/dev/ttyUSB0";
  int baud_rate = 115200;
  
  // 如果提供了命令行参数，则使用参数指定的端口
  if (argc > 1) {
    port = argv[1];
  }
  
  std::cout << "Attempting to connect to Roomba on " << port << " with baud rate " << baud_rate << std::endl;
  
  // 创建Roomba硬件实例
  auto roomba = std::make_unique<roomba_hardware_interface::RoombaHardware>(port, baud_rate);
  
  // 尝试连接
  if (!roomba->connect()) {
    std::cerr << "Failed to connect to Roomba on " << port << std::endl;
    return -1;
  }
  
  std::cout << "Successfully connected to Roomba!" << std::endl;
  
  // 启动并设置为安全模式
  roomba->start();
  std::this_thread::sleep_for(100ms);
  roomba->safe_mode();
  std::this_thread::sleep_for(100ms);
  
  std::cout << "Roomba set to Safe Mode" << std::endl;
  
  // 发送一个简单的驱动命令（向前移动）
  std::cout << "Sending drive command (100mm/s forward)..." << std::endl;
  roomba->drive_direct(100, 100); // 左右轮都以100mm/s速度前进
  std::this_thread::sleep_for(2s); // 持续2秒
  
  // 停止
  std::cout << "Stopping..." << std::endl;
  roomba->drive_direct(0, 0); // 停止运动
  
  // 读取一些传感器数据
  std::cout << "Reading sensor data:" << std::endl;
  std::cout << "  Battery charge: " << roomba->read_battery_charge() << " mAh" << std::endl;
  std::cout << "  Battery capacity: " << roomba->read_battery_capacity() << " mAh" << std::endl;
  std::cout << "  Charging state: " << roomba->get_charging_state_description() << std::endl;
  std::cout << "  Temperature: " << static_cast<int>(roomba->read_temperature()) << " degrees Celsius" << std::endl;
  
  // 断开连接
  roomba->stop();
  roomba->disconnect();
  
  std::cout << "Disconnected from Roomba" << std::endl;
  
  return 0;
}