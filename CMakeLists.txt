cmake_minimum_required(VERSION 3.8)
project(tcp_ip_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(SOCKETS_SRC
  src/sockets/tcp/client.cpp
  src/sockets/tcp/server.cpp
  src/sockets/tcp/session.cpp
  src/sockets/udp/broadcaster.cpp
  src/sockets/udp/listener.cpp
  src/sockets/address.cpp
  src/sockets/receiver.cpp
  src/sockets/sender.cpp
  src/sockets/socket.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOCKETS_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(sensor_msgs_msg_laser_scan_tcp_server src/nodes/sensor_msgs/msg/laser_scan/tcp/server.cpp)
ament_target_dependencies(sensor_msgs_msg_laser_scan_tcp_server
  rclcpp
  std_msgs
)

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(TARGETS ${PROJECT_NAME} sensor_msgs_msg_laser_scan_tcp_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
