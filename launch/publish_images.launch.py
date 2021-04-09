#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import LogInfo, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
import yaml
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    aruco_config = LaunchConfiguration('aruco_config')
    config_filepath = LaunchConfiguration('config_filepath')
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('aruco_config',default_value='sim'),
        DeclareLaunchArgument('namespace',default_value="drone_0"),
        DeclareLaunchArgument('config_filepath', default_value=[
            TextSubstitution(text=os.path.join(get_package_share_directory('aruco_ros'),'config','')),
            aruco_config,
            TextSubstitution(text='.yml')]
        ),
        Node(
            package='aruco_ros', executable='aruco_detector',
            output='screen',
            namespace=namespace,
            parameters=[
                config_filepath
            ],
        ),
        Node(
            package='aruco_ros', executable='landing_tf',
            output='screen',
            namespace=namespace,
            parameters=[
                config_filepath
            ]
        ),
        LogInfo(msg=[
            'Launching ', LaunchConfiguration('aruco_config'), ' aruco_detector'
        ])
    ])
