set(_AMENT_PACKAGE_NAME "ros2_pkg")
set(ros2_pkg_VERSION "0.0.0")
set(ros2_pkg_MAINTAINER "parallels <parallels@todo.todo>")
set(ros2_pkg_BUILD_DEPENDS "rosidl_default_generators" "rclcpp" "std_msgs" "sensor_msgs" "OpenCV" "cv_bridge" "geometry_msgs" "action_msgs")
set(ros2_pkg_BUILDTOOL_DEPENDS "ament_cmake")
set(ros2_pkg_BUILD_EXPORT_DEPENDS "rclcpp" "std_msgs" "sensor_msgs" "OpenCV" "cv_bridge" "geometry_msgs" "action_msgs")
set(ros2_pkg_BUILDTOOL_EXPORT_DEPENDS )
set(ros2_pkg_EXEC_DEPENDS "rosidl_default_runtime" "rclcpp" "std_msgs" "sensor_msgs" "OpenCV" "cv_bridge" "geometry_msgs" "action_msgs")
set(ros2_pkg_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(ros2_pkg_GROUP_DEPENDS )
set(ros2_pkg_MEMBER_OF_GROUPS "rosidl_interface_packages")
set(ros2_pkg_DEPRECATED "")
set(ros2_pkg_EXPORT_TAGS)
list(APPEND ros2_pkg_EXPORT_TAGS "<build_type>ament_cmake</build_type>")