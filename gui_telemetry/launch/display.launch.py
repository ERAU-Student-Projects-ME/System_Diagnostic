from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='gui_telemetry', executable='publisher', output='screen'),
        Node(package='gui_telemetry', executable='gui_display', output='screen')
    ])