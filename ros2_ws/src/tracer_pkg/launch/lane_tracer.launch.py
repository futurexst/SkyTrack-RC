from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log = LaunchConfiguration('log')

    return LaunchDescription([
        DeclareLaunchArgument(
            'log',
            default_value='false',
            description='Enable video_logger_node and multi_logger_node together'
        ),

        Node(
            package='tracer_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen',
        ),

        Node(
            package='tracer_pkg',
            executable='perception_node',
            name='perception_node',
            output='screen',
        ),

        Node(
            package='tracer_pkg',
            executable='video_logger_node',
            name='video_logger_node',
            output='screen',
            condition=IfCondition(log),
        ),

        Node(
            package='tracer_pkg',
            executable='multi_logger_node',
            name='multi_logger_node',
            output='screen',
            condition=IfCondition(log),
        ),
    ])
