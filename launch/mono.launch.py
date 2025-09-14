import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('orin_rp2_csi')
    
    mono_params = os.path.join(pkg_share_dir, 'params', 'mono.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'sensor_id',
            default_value='0',
            description='Sensor ID for the camera (0 or 1)'
        ),
        DeclareLaunchArgument(
            'display_mode',
            default_value='none',
            description='Display mode: none or image'
        ),
        # Run the mono camera processing node
        Node(
            package='orin_rp2_csi',
            executable='mono_processor',
            name='mono_processor',
            output='screen',
            parameters=[mono_params, {'sensor_id': LaunchConfiguration('sensor_id'), 'display_mode': LaunchConfiguration('display_mode')}]
        )
    ])