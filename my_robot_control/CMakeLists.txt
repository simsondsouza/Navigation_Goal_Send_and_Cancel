cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executables
add_executable(goal_sender src/send_goal.cpp)
add_executable(goal_cancel src/goal_cancel.cpp)

# Link libraries and set dependencies for each executable
ament_target_dependencies(goal_sender
    rclcpp
    rclcpp_action
    nav2_msgs
    geometry_msgs
)

ament_target_dependencies(goal_cancel
    rclcpp
    rclcpp_action
    nav2_msgs
    geometry_msgs
)

# Install executables
install(TARGETS
  goal_sender
  goal_cancel
  DESTINATION lib/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install dependencies
ament_export_dependencies(ament_cmake)
ament_package()

