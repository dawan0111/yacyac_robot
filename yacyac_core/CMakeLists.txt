cmake_minimum_required(VERSION 3.5)
project(yacyac_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yacyac_interface REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(
  ${PROJECT_NAME}_lib SHARED
  src/qr_client.cpp
  src/message.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp std_msgs behaviortree_cpp yacyac_interface
)

add_executable(talker src/pub.cpp)
add_executable(listener src/sub.cpp)
add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs behaviortree_cpp yacyac_interface)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  bt_tree
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  talker
  listener
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()