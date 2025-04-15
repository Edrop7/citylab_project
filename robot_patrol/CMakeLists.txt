cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(action_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


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

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPose.action"
  DEPENDENCIES geometry_msgs
)

add_executable(patrol_node src/patrol.cpp)
add_executable(direction_service_node src/direction_service.cpp)
add_executable(test_client_node src/test_service.cpp)
add_executable(patrol_with_service_node src/patrol_with_service.cpp)
add_executable(go_to_pose_node src/go_to_pose_action.cpp)

ament_target_dependencies(patrol_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

ament_target_dependencies(direction_service_node
  rclcpp
  sensor_msgs
  custom_interfaces
)

ament_target_dependencies(test_client_node
  rclcpp
  sensor_msgs
  custom_interfaces
)

ament_target_dependencies(patrol_with_service_node
  rclcpp
  sensor_msgs
  geometry_msgs
  custom_interfaces
)

ament_target_dependencies(go_to_pose_node
  rclcpp
  rclcpp_action
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

rosidl_target_interfaces(go_to_pose_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  patrol_node
  direction_service_node
  test_client_node
  patrol_with_service_node
  go_to_pose_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)


ament_package()
