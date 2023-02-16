import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace
from ament_index_python import get_package_share_directory
import os

sllidar_param_file = os.path.join(
  c1tenth_launch_pkg_prefix, 'param/sllidar.param.yaml')

sllidar_wrapper_param_file = os.path.join(
  c1tenth_launch_pkg_prefix, 'param/sllidar.param.yaml')

def generate_launch_description():
  sllidar_node = Node(
          package='sllidar_ros2',
          executable='sllidar_node',
          name='sllidar_node',
          namespace='lidar',
          parameters=[sllidar_driver_param_file],
          output='screen',
          remappings=[
              ('scan', 'points_raw')
              
              ])

  sllidar_ros2_driver_wrapper_node = Node(
          package='sllidar_ros2_driver_wrapper',
          executable='sllidar_ros2_driver_wrapper_node_exec',
          name='sllidar_ros2_driver_wrapper_node',
          namespace='lidar',
          parameters=[sllidar_wrapper_param_file],
          output='screen'
          
          )

    return launch.LaunchDescription([
        sllidar_node,
        sllidar_ros2_driver_wrapper_node
        
        ])
