cmake_minimum_required(VERSION 3.8)
project(wormhole_nav)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Find SQLite3
find_library(SQLITE3_LIB sqlite3)
include_directories(/usr/include)

add_executable(wormhole_server scripts/wormhole_server.cpp)
ament_target_dependencies(wormhole_server rclcpp rclcpp_action nav2_msgs geometry_msgs my_robot_interfaces)

add_executable(wormhole_client scripts/wormhole_client.cpp)
ament_target_dependencies(wormhole_client rclcpp rclcpp_action nav2_msgs geometry_msgs my_robot_interfaces)

# Link SQLite3
target_link_libraries(sql_node ${SQLITE3_LIB})
target_link_libraries(wormhole_server ${SQLITE3_LIB})


install(TARGETS

  wormhole_server
  wormhole_client

  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
