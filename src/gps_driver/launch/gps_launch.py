import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import glob

def detect_serial_port():
    """ Automatically finds the first available serial port. """
    possible_ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/pts/*')
    return possible_ports[0] if possible_ports else '/dev/ttyUSB0'

def generate_launch_description():
    # Declare a launch argument to pass the GPS serial port
    serial_port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',  # Change this if using an emulator
        description='Serial port for the GPS device'
    )

    # Define the GPS driver node
    gps_driver_node = Node(
        package='gps_driver',
        executable='gps_driver',
        name='gps_driver',
        output='screen',
        parameters=[{'port': LaunchConfiguration('port')}]
    )

    return LaunchDescription([
        serial_port_arg,
        gps_driver_node
    ])

