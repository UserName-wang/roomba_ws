cmake_minimum_required(VERSION 3.8)
project(roomba_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Find libserialport
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/roomba_hardware_driver.cpp
  src/roomba_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${LIBSERIALPORT_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} ${LIBSERIALPORT_LIBRARIES})

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Create example executable
add_executable(roomba_example
  src/roomba_example.cpp
  src/roomba_hardware_driver.cpp
)

target_include_directories(roomba_example PRIVATE
  include
  ${LIBSERIALPORT_INCLUDE_DIRS}
)

target_link_libraries(roomba_example ${LIBSERIALPORT_LIBRARIES})

ament_target_dependencies(roomba_example
  rclcpp
)

pluginlib_export_plugin_description_file(hardware_interface roomba_hardware_interface.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  TARGETS roomba_example
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES roomba_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

# EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()