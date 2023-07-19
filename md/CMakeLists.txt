cmake_minimum_required(VERSION 3.5)
project(md)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(yacyac_interface REQUIRED)
find_package(tf2_ros REQUIRED)


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

include_directories(include)



add_executable(md src/main.cpp)
ament_target_dependencies(md 
  rclcpp 
  std_msgs 
  yacyac_interface 
  geometry_msgs 
  tf2 
  tf2_ros
  nav_msgs 
  sensor_msgs 
  serial
  )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION $share/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.hpp"
# )

install(TARGETS
  md
  DESTINATION lib/${PROJECT_NAME})

install (DIRECTORY
  launch param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()