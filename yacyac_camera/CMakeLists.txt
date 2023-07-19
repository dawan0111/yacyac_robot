cmake_minimum_required(VERSION 3.8)
project(yacyac_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yacyac_interface REQUIRED)
find_library(ZBAR_LIBRARIES NAMES zbar)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node
  rclcpp
  std_msgs
  cv_bridge
  OpenCV
)

add_executable(qr_detector_node src/qr_detector_node.cpp)
ament_target_dependencies(qr_detector_node
  rclcpp
  std_msgs
  cv_bridge
  OpenCV
  yacyac_interface
)
target_link_libraries(qr_detector_node zbar)

add_executable(camera_qr_pipeline src/camera_qr_pipeline.cpp)
ament_target_dependencies(camera_qr_pipeline
  rclcpp
  std_msgs
  cv_bridge
  OpenCV
  yacyac_interface
)
target_link_libraries(camera_qr_pipeline zbar)

install(TARGETS
  camera_node
  qr_detector_node
  camera_qr_pipeline
  DESTINATION lib/${PROJECT_NAME})

install (DIRECTORY
  launch param
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
