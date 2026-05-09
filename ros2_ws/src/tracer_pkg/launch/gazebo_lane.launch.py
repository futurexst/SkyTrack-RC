#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('tracer_pkg')

    world_path = os.path.join(pkg_share, 'worlds', 'lane_world.sdf')
    model_path = os.path.join(pkg_share, 'models')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=model_path
        ),

        ExecuteProcess(
            cmd=[
                'gz', 'sim',
                '-r',
                world_path
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                # ROS -> Gazebo
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',

                # Gazebo -> ROS
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            output='screen'
        ),
    ])