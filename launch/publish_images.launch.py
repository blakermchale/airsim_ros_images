#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace',default_value="drone_0"),
        Node(
            package='airsim_ros_images', executable='publish_images',
            output='screen',
            namespace=[namespace, "/realsense"],
        ),
    ])
