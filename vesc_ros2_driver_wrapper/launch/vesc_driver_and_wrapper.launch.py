
import os

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

VESC_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('vesc_ros2_driver_wrapper'), 'config', 'vesc.params.yaml')

ODOM_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('vesc_ros2_driver_wrapper'), 'config', 'vesc_to_odom_node.params.yaml')

JOY_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('vesc_ros2_driver_wrapper'), 'config', 'joy.params.yaml')

F710_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('vesc_ros2_driver_wrapper'), 'config', 'logitech_f710_basic.params.yaml')

WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('vesc_ros2_driver_wrapper'), 'config', 'wrapper.params.yaml')

def generate_launch_description():

  # Arguments to this launch file.

  composable = DeclareLaunchArgument(
    name = 'composable', default_value='false', description='Should we launch a composable node')

  with_joy = DeclareLaunchArgument(
    name = 'with_joy', default_value='true', description='Launch joystick_interface in addition to other nodes')

  # Note that the name must match the param file.
    
  vesc_driver_launcher = Node(
      package='vesc_driver',
      executable='vesc_driver_node',
      name='vesc_driver_node',
      namespace='vehicle',
      output='screen',
      parameters=[VESC_DRIVER_PARAM_FILE]
  )

  vesc_to_odom_launcher = Node(
      package='vesc_ackermann',
      executable='vesc_to_odom_node',
      name='vesc_to_odom_node',
      namespace='vehicle',
      output='screen',
      parameters=[ODOM_DRIVER_PARAM_FILE],
      remappings=[
          ('odom', "vesc_odom")
      ]
  )

  joy = Node(
      package='joy_linux',
      executable='joy_linux_node',
      output='screen',
      parameters=[JOY_DRIVER_PARAM_FILE]
  )

  joy_translator = Node(
      package='joystick_vehicle_interface_nodes',
      executable='joystick_vehicle_interface_node_exe',
      output='screen',
      parameters=[F710_DRIVER_PARAM_FILE],
      remappings=[
          ("basic_command", "/vehicle/vehicle_command"),
          ("auto_basic_command", "/vehicle/auto_vehicle_command"),
          ("state_command", "/vehicle/state_command")
      ],
      condition=IfCondition(LaunchConfiguration('with_joy'))
  )

  # If we want a regular node (composable:=false) then this is run.
  wrapper_node = Node(
          name='vesc_ros2_driver_wrapper_node',
          package='vesc_ros2_driver_wrapper',
          executable='vesc_driver_wrapper_node',
          parameters=[WRAPPER_PARAM_FILE],
          namespace='vesc',
          output='screen',
          condition=UnlessCondition(LaunchConfiguration("composable"))
      )

  # If we want a composable node (composable:=true) then this is run.
  wrapper_composable_node = ComposableNodeContainer(
          name='vesc_ros2_driver_wrapper_container',
          package='carma_ros2_utils',
          executable='carma_component_container_mt',
          namespace='vesc',
          condition=IfCondition(LaunchConfiguration("composable")),
          composable_node_descriptions=[
              ComposableNode(
                  name='vesc_ros2_driver_wrapper_composable_node',
                  package='vesc_ros2_driver_wrapper',
                  plugin='vesc_ros2_driver_wrapper::ComposableNode',
                  parameters=[WRAPPER_PARAM_FILE],
                  extra_arguments=[{'use_intra_process_comms': True}],
              )
          ]
      )
  
  # Must return arguments and nodes as a launch description. 
  return launch.LaunchDescription([
        composable,
        with_joy,
        vesc_driver_launcher,
        vesc_to_odom_launcher,
        joy,
        joy_translator,
        #wrapper_node,
        #wrapper_composable_node
    ])