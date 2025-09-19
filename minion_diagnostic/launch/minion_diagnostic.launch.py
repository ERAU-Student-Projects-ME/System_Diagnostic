from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    systeminfo_node = Node(
                            package='minion_diagnostic',
                            executable='systeminfo_node',
                            name='systeminfo_node',
                            output='screen',
                            parameters=[]
                        )
    gui_node = Node(
                    package='minion_diagnostic',
                    executable='system_health_monitor_node',
                    name='system_health_monitor_node',
                    output='screen',
                    parameters=[]
                )
    
    return LaunchDescription([
        systeminfo_node,
        gui_node
    ])