cmake_minimum_required(VERSION 3.5)
project(roomba_driver_test)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(roomba_hardware_interface REQUIRED)

# Simple Roomba test
add_executable(simple_roomba_test src/simple_roomba_test.cpp)
target_include_directories(simple_roomba_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(simple_roomba_test roomba_hardware_interface)

# Battery status test
add_executable(battery_status_test src/battery_status_test.cpp)
target_include_directories(battery_status_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(battery_status_test roomba_hardware_interface)

# Link the library
target_link_libraries(battery_status_test 
  ${roomba_hardware_interface_LIBRARIES})
  
target_link_libraries(simple_roomba_test 
  ${roomba_hardware_interface_LIBRARIES})

install(TARGETS 
  simple_roomba_test
  battery_status_test
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()