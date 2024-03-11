cmake_minimum_required(VERSION 3.8)
project(tcp_ip_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB_RECURSE LIBRARY_SRC 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/sockets/*.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/msgs/*.cpp
)

add_library(${PROJECT_NAME} SHARED ${LIBRARY_SRC})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(NODES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes)

file(GLOB_RECURSE NODES_SRC ${NODES_DIR}/*.cpp)
foreach(NODE_SRC ${NODES_SRC})
  file(RELATIVE_PATH RELATIVE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src/nodes ${NODE_SRC})
  string(REPLACE ".cpp" "" RELATIVE_PATH ${RELATIVE_PATH})
  string(REPLACE "/" "_" EXECUTABLE_NAME ${RELATIVE_PATH})

  add_executable(${EXECUTABLE_NAME} ${NODE_SRC})
  ament_target_dependencies(${EXECUTABLE_NAME}
    rclcpp
    std_msgs
  )
  target_link_libraries(${EXECUTABLE_NAME} ${PROJECT_NAME})

  install(TARGETS ${EXECUTABLE_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )
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
