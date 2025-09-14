import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('orin_rp2_csi')
    
    stereo_params = os.path.join(pkg_share_dir, 'params', 'stereo.yaml')
    
    return LaunchDescription([
        # Run the stereo camera processing node
        Node(
            package='orin_rp2_csi',
            executable='stereo_processor',
            name='stereo_processor',
            output='screen',
            parameters=[stereo_params, {'display_mode': 'side_by_side'}]
        )
    ])
