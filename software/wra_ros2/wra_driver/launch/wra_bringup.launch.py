"""ros2 launch wra_driver wra_bringup.launch.py port:=/dev/ttyUSB0 side:=right rviz:=false
Starts: robot_state_publisher (URDF), the WRA-1 driver, a static TF chair -> base_link for the
chosen side (from arm_geometry.json), optionally RViz.  Use port:=/dev/pts/N with the fake ESP32."""
import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def mount_tf(context):
    side = LaunchConfiguration("side").perform(context)
    g = json.load(open(os.path.join(get_package_share_directory("wra_driver"), "config", "arm_geometry.json")))
    m = g.get("mount", {}).get(side)
    if not m:
        return []
    return [Node(package="tf2_ros", executable="static_transform_publisher", name="chair_to_base",
                 arguments=["--x", str(m["xyz"][0]), "--y", str(m["xyz"][1]), "--z", str(m["xyz"][2]),
                            "--yaw", str(m["rpy"][2]), "--pitch", str(m["rpy"][1]), "--roll", str(m["rpy"][0]),
                            "--frame-id", "chair", "--child-frame-id", "base_link"])]


def generate_launch_description():
    desc = get_package_share_directory("wra_description")
    urdf = os.path.join(desc, "urdf", "wra.urdf.xacro")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("side", default_value="right"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("calibration", default_value=""),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": ParameterValue(
                 Command(["xacro ", urdf, " side:=", LaunchConfiguration("side")]), value_type=str)}]),
        Node(package="wra_driver", executable="driver_node", output="screen",
             parameters=[{"port": LaunchConfiguration("port"), "side": LaunchConfiguration("side"),
                          "calibration": LaunchConfiguration("calibration")}]),
        OpaqueFunction(function=mount_tf),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(desc, "rviz", "wra.rviz")]),
    ])
