cmake_minimum_required(VERSION 3.8)
project(minimal_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(minimal_services_client src/minimal_client.cpp)
ament_target_dependencies(minimal_services_client rclcpp example_interfaces)

add_executable(minimal_services_service src/minimal_service.cpp)
ament_target_dependencies(minimal_services_service rclcpp example_interfaces)

add_executable(minimal_async_client src/async_client.cpp)
ament_target_dependencies(minimal_async_client rclcpp example_interfaces)

install(TARGETS
  minimal_services_client
  minimal_services_service
  minimal_async_client
  DESTINATION lib/${PROJECT_NAME}
)


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
