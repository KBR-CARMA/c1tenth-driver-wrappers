
import os

from ament_index_python import get_package_share_directory
from carma_ros2_utils.launch.get_current_namespace import GetCurrentNamespace

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('bno055_ros2_driver_wrapper'), 'config', 'bno055_driver.params.yaml')

WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('bno055_ros2_driver_wrapper'), 'config', 'bno055_wrapper.params.yaml')

def generate_launch_description():

  # Arguments to this launch file.
  composable = DeclareLaunchArgument(
    name = 'composable', default_value='false', description='Should we launch a composable node')

  # Note that the name must match the param file.
  driver_node = Node(
          name='bno055_ros2_driver',
          package='bno055',
          executable='bno055',
          parameters=[DRIVER_PARAM_FILE],
          namespace=GetCurrentNamespace(),
          output='screen'
      )

  # If we want a regular node (composable:=false) then this is run.
  wrapper_node = Node(
          name='bno055_ros2_driver_wrapper',
          package='bno055_ros2_driver_wrapper',
          executable='bno055_driver_wrapper_node',
          parameters=[WRAPPER_PARAM_FILE],
          namespace=GetCurrentNamespace(),
          output='screen',
          condition=UnlessCondition(LaunchConfiguration("composable"))
      )

  # If we want a composable node (composable:=true) then this is run.
  wrapper_composable_node = ComposableNodeContainer(
          name='bno055_ros2_driver_wrapper_container',
          package='carma_ros2_utils',
          executable='carma_component_container_mt',
          namespace=GetCurrentNamespace(),
          condition=IfCondition(LaunchConfiguration("composable")),
          composable_node_descriptions=[
              ComposableNode(
                  name='bno055_ros2_driver_wrapper',
                  package='bno055_ros2_driver_wrapper',
                  plugin='bno055_ros2_driver_wrapper::ComposableNode',
                  parameters=[WRAPPER_PARAM_FILE],
                  extra_arguments=[{'use_intra_process_comms': True}],
              )
          ]
      )
  
  # Must return arguments and nodes as a launch description. 
  return launch.LaunchDescription([
        composable,
        driver_node,
        wrapper_node,
        wrapper_composable_node
    ])