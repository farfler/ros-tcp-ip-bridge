cmake_minimum_required(VERSION 3.8)
project(tcp_ip_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(builtin_interfaces REQUIRED)

file(GLOB_RECURSE TCP_IP_BRIDGE_LIB
  ${CMAKE_CURRENT_SOURCE_DIR}/src/msgs/*.cpp
)

file(GLOB_RECURSE TCP_IP_BRIDGE_NODES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes/*.cpp
)

add_library(${PROJECT_NAME} SHARED ${TCP_IP_BRIDGE_LIB})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  builtin_interfaces
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

foreach(TCP_IP_BRIDGE_NODE ${TCP_IP_BRIDGE_NODES})
  file(RELATIVE_PATH RELATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes ${TCP_IP_BRIDGE_NODE})
  string(REPLACE ".cpp" "" RELATIVE_PATH ${RELATIVE_PATH})
  string(REPLACE "/" "_" EXECUTABLE_NAME ${RELATIVE_PATH})

  add_executable(${EXECUTABLE_NAME} ${TCP_IP_BRIDGE_NODE})
  ament_target_dependencies(${EXECUTABLE_NAME}
    rclcpp
    builtin_interfaces
  )

  target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME})
  install(TARGETS ${EXECUTABLE_NAME} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(TARGETS ${PROJECT_NAME}
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
