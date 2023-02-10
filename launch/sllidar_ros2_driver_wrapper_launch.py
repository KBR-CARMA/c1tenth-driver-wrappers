import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory
import os

sllidar_param_file = os.path.join(
  c1tenth_launch_pkg_prefix, 'param/sllidar.param.yaml')

sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        namespace='lidar',
        parameters=[sllidar_param_file],
        output='screen',
        remappings=[
            ('scan', 'points_raw')
        ]
    )

return launch.LaunchDescription([
        sllidar_node
    ])
