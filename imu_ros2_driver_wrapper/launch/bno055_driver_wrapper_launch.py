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
# TODO: remapings, params, names
bno055_driver_param_file = os.path.join(
  c1tenth_launch_pkg_prefix, 'params/bno055.params.yaml')

sllidar_wrapper_param_file = os.path.join(
  c1tenth_launch_pkg_prefix, 'param/bno055_driver_wrapper.param.yaml')

def generate_launch_description():
  bno055_driver_node = Node(
          package='bno055',
          executable='bno055',
          name='sllidar_node',
          namespace='bno055',
          parameters=[bno055_driver_param_file],
          output='screen',
          remappings=[
              ('scan', 'points_raw')
          ]
      )

  bno055_driver_wrapper_node = Node(
          package='bno055_driver_wrapper',
          executable='bno055_driver_wrapper_node_exec',
          name='bno055_driver_wrapper_node',
          namespace='bno055',
          parameters=[bno055_wrapper_param_file],
          output='screen'
      )

  return launch.LaunchDescription([
        bno055_driver_node,
        bno055_driver_wrapper_node
    ])