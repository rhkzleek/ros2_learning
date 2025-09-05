cmake_minimum_required(VERSION 3.8)
project(minimal_action_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(action_client_member_functions src/member_functions.cpp)
ament_target_dependencies(action_client_member_functions
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

add_executable(action_client_not_composable src/not_composable.cpp)
ament_target_dependencies(action_client_not_composable
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

add_executable(action_client_not_composable_with_cancel src/not_composable_with_cancel.cpp)
ament_target_dependencies(action_client_not_composable_with_cancel
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

add_executable(action_client_not_composable_with_feedback src/not_composable_with_feedback.cpp)
ament_target_dependencies(action_client_not_composable_with_feedback
  "rclcpp"
  "rclcpp_action"
  "example_interfaces")

install(TARGETS
  action_client_member_functions
  action_client_not_composable
  action_client_not_composable_with_cancel
  action_client_not_composable_with_feedback
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
