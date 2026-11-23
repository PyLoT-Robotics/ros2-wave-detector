from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default=Path(get_package_share_directory('wave_detector'))/'config'/'params.yaml'
    return LaunchDescription([DeclareLaunchArgument('params_file',default_value=str(default)),
        Node(package='wave_detector',executable='wave_detector_node',name='wave_detector',
             parameters=[LaunchConfiguration('params_file')],output='screen')])
