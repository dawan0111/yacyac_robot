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
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(yacyac_interface REQUIRED)
find_package(tf2_ros REQUIRED)


set(DEPENDENCIES
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav2_msgs
  behaviortree_cpp
  yacyac_interface
  ${OTHER_DEPS}
)  


add_library(
  ${PROJECT_NAME}_lib SHARED
  src/qr_client.cpp
  src/message.cpp
  src/nav2_client.cpp
)

# send_goal
add_executable(send_goal 
  src/send_goal.cpp
)
ament_target_dependencies(send_goal
	${DEPENDENCIES}
)
# -----------------------------

# bt_ros2
add_executable(bt_ros2
  src/bt_ros2.cpp
)
# ament_target_dependencies(bt_ros2
# 	${DEPENDENCIES}
# )
#  -----------------------------


# add_executable(send_goal src/send_goal.cpp)
add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp std_msgs behaviortree_cpp yacyac_interface nav2_msgs
)

ament_target_dependencies(send_goal rclcpp std_msgs rclcpp_action geometry_msgs nav2_msgs yacyac_interface)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
target_link_libraries(bt_ros2 ${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  bt_xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  send_goal
  bt_ros2
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
