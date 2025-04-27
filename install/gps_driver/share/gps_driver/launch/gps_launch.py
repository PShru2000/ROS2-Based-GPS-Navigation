import os
from launch import LaunchDescription
from launch_ros.actions import Node
import glob

# Inline serial port detection
available_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/pts/*')
serial_port = available_ports[0] if available_ports else '/dev/ttyUSB0'

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_driver',
            executable='gps_driver',
            name='gps_publisher',
            output='screen',
            parameters=[{'port': serial_port}]
        )
    ])

