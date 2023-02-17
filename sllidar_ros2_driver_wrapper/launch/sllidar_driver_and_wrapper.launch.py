
import os

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('sllidar_ros2_driver_wrapper'), 'config', 'sllidar_driver.params.yaml')

WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('sllidar_ros2_driver_wrapper'), 'config', 'sllidar_wrapper.params.yaml')

def generate_launch_description():

  # Arguments to this launch file.
  composable = DeclareLaunchArgument(
    name = 'composable', default_value='false', description='Should we launch a composable node')

  # Note that the name must match the param file.
  sllidar_node = Node(
    package='sllidar_ros2',
    executable='sllidar_node',
    name='sllidar_node',
    namespace='lidar',
    parameters=[DRIVER_PARAM_FILE],
    output='screen',
    remappings=[('scan', 'points_raw')])
  
  # If we want a regular node (composable:=false) then this is run.
  wrapper_node = Node(
    name='sllidar_driver_wrapper_node',
    package='sllidar_ros2_driver_wrapper',
    executable='sllidar_driver_wrapper_node',
    parameters=[WRAPPER_PARAM_FILE],
    namespace='lidar',
    output='screen',
    condition=UnlessCondition(LaunchConfiguration("composable")))

  # If we want a composable node (composable:=true) then this is run.
  wrapper_composable_node = ComposableNodeContainer(
    name='sllidar_driver_wrapper_node',
    package='carma_ros2_utils',
    executable='carma_component_container_mt',
    namespace='lidar',
    condition=IfCondition(LaunchConfiguration("composable")),
    composable_node_descriptions=[
      ComposableNode(
        name='sllidar_ros2_driver_wrapper_composable_node',
        package='sllidar_ros2_driver_wrapper',
        plugin='sllidar_ros2_driver_wrapper::ComposableNode',
        parameters=[WRAPPER_PARAM_FILE],
        extra_arguments=[{'use_intra_process_comms': True}],
      )
    ])
  
  # Must return arguments and nodes as a launch description. 
  return launch.LaunchDescription([
        composable,
        driver_node,
        wrapper_node,
        wrapper_composable_node
    ])