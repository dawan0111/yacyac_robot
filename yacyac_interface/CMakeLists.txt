cmake_minimum_required(VERSION 3.8)
project(yacyac_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(rclcpp REQUIRED)

# 헤더 파일 include
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/insta360)
# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

# 노드 추가
# add_executable(yacyac_ctrl src/pub.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/Pose.msg"
  DEPENDENCIES std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)

# ament_target_dependencies(yacyac_ctrl
#   "rclcpp"
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
