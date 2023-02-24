
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
  get_package_share_directory('joy_ros2_driver_wrapper'), 'config', 'joy_driver.params.yaml')

WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('joy_ros2_driver_wrapper'), 'config', 'joy_wrapper.params.yaml')

def generate_launch_description():

  # Arguments to this launch file.
  composable = DeclareLaunchArgument(
    name = 'composable', default_value='false', description='Should we launch a composable node')

  # Publishes:
  # + joy                             (sensor_msgs::msg::Joy)
  driver_node = Node(
      package='joy_linux',
      executable='joy_linux_node',
      output='screen',
      namespace='input',
      parameters=[DRIVER_PARAM_FILE]
  )

  # Subscribes to:
  # + joy                             (sensor_msgs::msg::Joy)
  # Publishes:
  # + vehicle_cmd                     (autoware_msgs::msg::VehicleCmd)
  # + vehicle/engage                  (std_msgs::msg::Bool)
  wrapper_node = Node(
          name='joy_driver_wrapper_node',
          package='joy_ros2_driver_wrapper',
          executable='joy_driver_wrapper_node',
          parameters=[WRAPPER_PARAM_FILE],
          output='screen',
          namespace='',
          condition=UnlessCondition(LaunchConfiguration("composable"))
      )
  wrapper_composable_node = ComposableNodeContainer(
          name='joy_ros2_driver_wrapper_container',
          package='carma_ros2_utils',
          executable='carma_component_container_mt',
          namespace='',
          condition=IfCondition(LaunchConfiguration("composable")),
          composable_node_descriptions=[
              ComposableNode(
                  name='joy_ros2_driver_wrapper_composable_node',
                  package='joy_ros2_driver_wrapper',
                  plugin='joy_ros2_driver_wrapper::ComposableNode',
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