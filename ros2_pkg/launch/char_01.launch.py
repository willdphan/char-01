from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo',
                 '/media/psf/Developer/Robotics/char-01/ros2_pkg/worlds/char_01_world.sdf'],
            output='screen'
        ),
        Node(
            package="ros2_pkg",
            executable="predict.py",
            name="predict_node",
            output='screen'
        ),
        Node(
            package="ros2_pkg",
            executable="control",
            name="control_node",
            output='screen'
        ),
        Node(
            package="ros2_pkg",
            executable="lidar_node",
            name="lidar_node",
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
            output='screen'
        )
    ])
