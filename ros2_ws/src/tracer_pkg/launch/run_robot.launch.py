import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 카메라 노드 실행
        Node(
            package='tracer_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        
        # 2. 퍼셉션 노드 실행
        Node(
            package='tracer_pkg',
            executable='perception_node',
            name='perception_node',
            output='screen'
        ),
        
        # 3. 모터 제어 노드 실행
        Node(
            package='tracer_pkg',
            executable='control_node',
            name='control_node',
            output='screen'
        )
    ])