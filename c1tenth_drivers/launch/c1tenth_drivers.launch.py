
import os

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# BNO055_DRIVER_PARAM_FILE = os.path.join(
#   get_package_share_directory('c1tenth_drivers'), 'config', 'bno055_driver.params.yaml')
# BNO055_WRAPPER_PARAM_FILE = os.path.join(
#   get_package_share_directory('c1tenth_drivers'), 'config', 'bno055_wrapper.params.yaml')

# SLLIDAR_DRIVER_PARAM_FILE = os.path.join(
#   get_package_share_directory('c1tenth_drivers'), 'config', 'sllidar_driver.params.yaml')
# SLLIDAR_WRAPPER_PARAM_FILE = os.path.join(
#   get_package_share_directory('c1tenth_drivers'), 'config', 'sllidar_wrapper.params.yaml')

JOY_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'joy_driver.params.yaml')
JOY_WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'joy_wrapper.params.yaml')

VESC_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'vesc_driver.params.yaml')
VESC_WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'vesc_wrapper.params.yaml')

def generate_launch_description():

  ############# JOY

  # Publishes:
  # + joy                             (sensor_msgs::msg::Joy)
  joy_driver_node = Node(
      package='joy_linux',
      executable='joy_linux_node',
      output='screen',
      parameters=[JOY_DRIVER_PARAM_FILE]
  )

  # Subscribes to:
  # + joy                             (sensor_msgs::msg::Joy)
  # Publishes:
  # + vehicle_cmd                     (autoware_msgs::msg::VehicleCmd)
  # + vehicle/engage                  (std_msgs::msg::Bool)
  joy_wrapper_node = Node(
          name='joy_driver_wrapper_node',
          package='joy_ros2_driver_wrapper',
          executable='joy_driver_wrapper_node',
          parameters=[JOY_WRAPPER_PARAM_FILE],
          output='screen',
      )

  ############# VESC
    
  # Subscribes to:
  # + commands/motor/duty_cycle        (std_msgs::msg::Float64)
  # + commands/motor/current           (std_msgs::msg::Float64)
  # + commands/motor/brake             (std_msgs::msg::Float64)
  # + commands/motor/speed             (std_msgs::msg::Float64)
  # + commands/motor/position          (std_msgs::msg::Float64)
  # + commands/servo/position          (std_msgs::msg::Float64)
  # Publishes: 
  # + sensors/core                    (vesc_msgs::msg::VescStateStamped)
  # + sensors/imu                     (vesc_msgs::msg::VescImuStamped)
  # + sensors/imu/raw                 (sensor_msgs::msg::Imu)
  # + sensors/servo_position_command  (std_msgs::msg::Float64)
  vesc_driver_node = Node(
      package='vesc_driver',
      executable='vesc_driver_node',
      name='vesc_driver_node',
      output='screen',
      parameters=[VESC_DRIVER_PARAM_FILE]
  )

  # Subscribes to:
  # + sensors/core                    (vesc_msgs::msg::VescStateStamped)
  # + sensors/servo_position_command  (std_msgs::msg::Float64)
  # + vehicle_cmd                     (autoware_msgs::msg::VehicleCmd)
  # + vehicle/engage                  (std_msgs::msg::Bool)
  # Publishes:
  # + vehicle_status                  (autoware_msgs::msg::VehicleStatus)
  # + vehicle/twist                   (geometry_msgs::msg::TwistStamped)
  # + commands/motor/speed            (std_msgs::msg::Float64)
  # + commands/servo/position         (std_msgs::msg::Float64)
  vesc_wrapper_node = Node(
          name='vesc_ros2_driver_wrapper_node',
          package='vesc_ros2_driver_wrapper',
          executable='vesc_driver_wrapper_node',
          parameters=[VESC_WRAPPER_PARAM_FILE],
          output='screen',
      )
  
  # Must return arguments and nodes as a launch description. 
  return launch.LaunchDescription([
        joy_driver_node,
        joy_wrapper_node,
        vesc_driver_node,
        vesc_wrapper_node,
    ])