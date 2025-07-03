from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_serial_commander',
            executable='serial_commander',
            name='serial_commander',    # You can customize the node name if you want
            output='screen',
            parameters=[],
        ),
    ])
