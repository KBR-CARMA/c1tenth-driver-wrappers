
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
  get_package_share_directory('vesc_ros2_driver_wrapper'), 'config', 'vesc_driver.params.yaml')

WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('vesc_ros2_driver_wrapper'), 'config', 'vesc_wrapper.params.yaml')


def generate_launch_description():

  # Arguments to this launch file.

  composable = DeclareLaunchArgument(
    name = 'composable', default_value='false', description='Should we launch a composable node')

  # Note that the name must match the param file.
    
  # Subscribes to:
  # + vesc/commands/motor/duty_cycle        (std_msgs::msg::Float64)
  # + vesc/commands/motor/current           (std_msgs::msg::Float64)
  # + vesc/commands/motor/brake             (std_msgs::msg::Float64)
  # + vesc/commands/motor/speed             (std_msgs::msg::Float64)
  # + vesc/commands/motor/position          (std_msgs::msg::Float64)
  # + vesc/commands/servo/position          (std_msgs::msg::Float64)
  # Publishes: 
  # + vesc/sensors/core                     (vesc_msgs::msg::VescStateStamped)
  # + vesc/sensors/imu                      (vesc_msgs::msg::VescImuStamped)
  # + vesc/sensors/imu/raw                  (sensor_msgs::msg::Imu)
  # + vesc/sensors/servo_position_command   (std_msgs::msg::Float64)
  driver_node = Node(
      package='vesc_driver',
      executable='vesc_driver_node',
      name='vesc_driver_node',
      namespace='vesc',
      output='screen',
      parameters=[DRIVER_PARAM_FILE]
  )

  # Subscribes to:
  # + vesc/sensors/core                     (vesc_msgs::msg::VescStateStamped)
  # + vesc/sensors/servo_position_command   (std_msgs::msg::Float64)
  # + /vehicle_cmd                          (autoware_msgs::msg::VehicleCmd)
  # + /vehicle/engage                       (std_msgs::msg::Bool)
  # Publishes:
  # + vesc/commands/motor/speed             (std_msgs::msg::Float64)
  # + vesc/commands/servo/position          (std_msgs::msg::Float64)
  # + controller/vehicle_status             (autoware_msgs::msg::VehicleStatus)
  # + controller/vehicle/twist              (geometry_msgs::msg::TwistStamped)
  wrapper_node = Node(
          name='vesc_ros2_driver_wrapper_node',
          package='vesc_ros2_driver_wrapper',
          executable='vesc_driver_wrapper_node',
          parameters=[WRAPPER_PARAM_FILE],
          namespace=GetCurrentNamespace(),
          output='screen',
          condition=UnlessCondition(LaunchConfiguration("composable"))
      )
  wrapper_composable_node = ComposableNodeContainer(
          name='vesc_ros2_driver_wrapper_container',
          package='carma_ros2_utils',
          executable='carma_component_container_mt',
          namespace=GetCurrentNamespace(),
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
        driver_node,
        wrapper_node,
        wrapper_composable_node
    ])