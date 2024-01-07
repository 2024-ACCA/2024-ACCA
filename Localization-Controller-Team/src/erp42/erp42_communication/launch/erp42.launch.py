from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    erp_port = LaunchConfiguration('erp_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'erp_port',
            default_value='/dev/ttyUSB0',
            description='ERP port'
        ),

        Node(
            package='erp42_communication',
            executable='erp42_serial.py',
            name='erp42_serial',
            output='screen',
            parameters=[{'erp_port': erp_port}],
        ),
    ])
