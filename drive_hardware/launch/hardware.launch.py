from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    port_name = LaunchConfiguration('port_name')
    baud_rate = LaunchConfiguration('baud_rate')
    timeout_sec = LaunchConfiguration('timeout_sec')

    serial_driver_node = Node(
        package='drive_hardware',
        executable='rover_serial_driver',
        name='rover_serial_driver',
        output='screen',
        parameters=[
            {'port_name': port_name},
            {'baud_rate': baud_rate},
            {'timeout_sec': timeout_sec}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200'
        ),
        DeclareLaunchArgument(
            'timeout_sec',
            default_value='0.5'
        ),
        serial_driver_node
    ])

