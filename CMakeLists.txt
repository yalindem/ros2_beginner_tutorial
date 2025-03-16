cmake_minimum_required(VERSION 3.8)
project(ros2_beginner_tutorial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CountUntil.action"
)


add_executable(my_first_cpp_node src/my_first_node.cpp)
add_executable(robot_news_station_node src/robot_news_station.cpp)
add_executable(smartphone_node src/smartphone.cpp)
add_executable(add_two_ints_server_node src/add_two_ints_server.cpp)
add_executable(add_two_ints_client_no_oop_node src/add_two_ints_client_no_oop.cpp)
add_executable(add_two_ints_client_node src/add_two_ints_client.cpp)
add_executable(count_until_server_node src/count_until_server.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(count_until_server_node "${cpp_typesupport_target}")
 
ament_target_dependencies(my_first_cpp_node rclcpp)
ament_target_dependencies(robot_news_station_node rclcpp example_interfaces)
ament_target_dependencies(smartphone_node rclcpp example_interfaces)
ament_target_dependencies(add_two_ints_server_node rclcpp example_interfaces)
ament_target_dependencies(add_two_ints_client_no_oop_node rclcpp example_interfaces)
ament_target_dependencies(add_two_ints_client_node rclcpp example_interfaces)
ament_target_dependencies(count_until_server_node rclcpp rclcpp_action)


install(
  TARGETS 
  my_first_cpp_node
  robot_news_station_node
  smartphone_node
  add_two_ints_server_node
  add_two_ints_client_no_oop_node
  add_two_ints_client_node
  count_until_server_node
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
