cmake_minimum_required(VERSION 3.8)
project(minion_diagnostic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(minion_diagnostic_msg REQUIRED)

add_executable(system_monitor_node src/monitor_nodes_topics.cpp)
ament_target_dependencies(system_monitor_node
  rclcpp
  minion_diagnostic_msg
)

# Install executables
install(
  TARGETS system_monitor_node 
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  minion_diagnostic/system_health_monitor.py
  minion_diagnostic/systeminfo.py
  DESTINATION lib/${PROJECT_NAME}
)

install( 
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
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
