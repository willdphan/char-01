cmake_minimum_required(VERSION 3.8)
# reference name passed down to ${PROJECT_NAME}
# if u want to change name, change here AND the folder name
project(ros2_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# Generate custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/OddEvenCheck.srv"
  "srv/TurnCamera.srv"
  "action/Navigate.action"
  DEPENDENCIES
  sensor_msgs 
  geometry_msgs 
  action_msgs
  ADD_LINTER_TESTS
)

# Set support for using custom interfaces in C++ from this package
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Create an executable named 'publisher' from the source file 
# 'publisher.cpp' located in '/src/' directory.
add_executable(publisher /media/psf/Developer/Robotics/char-01/ros2_pkg/src/publisher.cpp)
# Specify the dependencies that the 'publisher.cpp'
# executable needs from ament package.
ament_target_dependencies(publisher rclcpp std_msgs)


add_executable(subscriber src/subscriber.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs)

add_executable(rpm_pub src/rpm_pub.cpp)
ament_target_dependencies(rpm_pub rclcpp std_msgs)

add_executable(speed_calc src/speed_calc.cpp)
ament_target_dependencies(speed_calc rclcpp std_msgs)

add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp std_msgs)
target_link_libraries(service_server "${cpp_typesupport_target}")

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp std_msgs)
target_link_libraries(service_client "${cpp_typesupport_target}")

add_executable(turn_camera_service src/turn_camera_service.cpp)
ament_target_dependencies(turn_camera_service rclcpp std_msgs sensor_msgs OpenCV cv_bridge)
target_link_libraries(turn_camera_service "${cpp_typesupport_target}")

add_executable(turn_camera_client src/turn_camera_client.cpp)
ament_target_dependencies(turn_camera_client rclcpp std_msgs sensor_msgs OpenCV cv_bridge)
target_link_libraries(turn_camera_client "${cpp_typesupport_target}")

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server rclcpp std_msgs sensor_msgs action_msgs rclcpp_action)
target_link_libraries(action_server "${cpp_typesupport_target}")

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client rclcpp std_msgs sensor_msgs action_msgs rclcpp_action)
target_link_libraries(action_client "${cpp_typesupport_target}")

add_executable(lidar_node sensors/lidar_node.cc)
ament_target_dependencies(lidar_node rclcpp std_msgs sensor_msgs)
target_link_libraries(lidar_node "${cpp_typesupport_target}")

add_executable(control src/control.cpp)
ament_target_dependencies(control rclcpp std_msgs geometry_msgs)
target_link_libraries(control "${cpp_typesupport_target}")


# config for ROS2 cli tools  
install(TARGETS 
  publisher 
  subscriber
  rpm_pub
  speed_calc
  service_server
  service_client
  turn_camera_service
  turn_camera_client
  action_server
  action_client
  lidar_node
  control
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/predict.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_ cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
