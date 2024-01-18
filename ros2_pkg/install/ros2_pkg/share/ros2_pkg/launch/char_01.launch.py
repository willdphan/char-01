from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', '/Users/williamphan/Desktop/Developer/Robotics/char-01/ros2_pkg/worlds/char_01_world.sdf'],
            output='screen'
        ),
        Node(
            package="ros2_pkg",
            executable="speed_calc",
            name="speed_calc_node",
            parameters=[
                {"wheel_radius": 10/100}  # Centimeters to Meters
            ]
        ),
        Node(
            package="ros2_pkg",
            executable="predict.py",
            name="predictor_node",
            output='screen'
        ),
        Node(
            package="ros2_pkg",
            executable="control",
            name="control_node",
            output='screen'
        ),
    ])
