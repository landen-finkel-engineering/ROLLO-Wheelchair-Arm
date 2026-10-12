"""ros2 launch wra_description display.launch.py  - URDF in RViz with joint sliders (no hardware)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    d = get_package_share_directory("wra_description")
    desc = ParameterValue(Command(["xacro ", os.path.join(d, "urdf", "wra.urdf.xacro")]), value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": desc}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(d, "rviz", "wra.rviz")]),
    ])
