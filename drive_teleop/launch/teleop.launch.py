from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    right_axis_index = LaunchConfiguration('right_axis_index')
    left_axis_index  = LaunchConfiguration('left_axis_index')
    drive_mode       = LaunchConfiguration('drive_mode')
    joy_dev          = LaunchConfiguration('joy_device')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='mobility_joy_node',
        output='screen',
        parameters=[
            {'device_name':     joy_dev},
            {'deadzone':        0.05},
            {'autorepeat_rate': 20.0},
        ],
        remappings=[
            ('joy', 'mobility_joy'),   # /joy → /mobility_joy  (arm ile çakışmaz)
        ],
    )

    tank_drive_node = Node(
        package='drive_teleop',
        executable='tank_drive_joystick',
        name='tank_drive_joystick',
        output='screen',
        parameters=[
            {'deadzone':        0.1},
            {'max_acceleration': 1.5},
            {'timeout_sec':     0.5},
            {'control_rate':    20.0},
            {'left_axis_index':  left_axis_index},
            {'right_axis_index': right_axis_index},
            {'drive_mode':       drive_mode},
        ],
        remappings=[
            ('joy', 'mobility_joy'),   # /joy → /mobility_joy
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_device',
            default_value='',
            description='Joystick cihaz adi (orn: /dev/input/js0). Bos: ilk bulunan.',
        ),
        DeclareLaunchArgument(
            'left_axis_index',
            default_value='1',
        ),
        DeclareLaunchArgument(
            'right_axis_index',
            default_value='4',
        ),
        DeclareLaunchArgument(
            'drive_mode',
            default_value='1',
        ),
        joy_node,
        tank_drive_node,
    ])
