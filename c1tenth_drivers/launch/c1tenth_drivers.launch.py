
import os

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

BNO055_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'bno055_driver.params.yaml')
BNO055_WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'bno055_wrapper.params.yaml')

SLLIDAR_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'sllidar_driver.params.yaml')
SLLIDAR_WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'sllidar_wrapper.params.yaml')

JOY_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'joy_driver.params.yaml')
JOY_WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'joy_wrapper.params.yaml')

VESC_DRIVER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'vesc_driver.params.yaml')
VESC_WRAPPER_PARAM_FILE = os.path.join(
  get_package_share_directory('c1tenth_drivers'), 'config', 'vesc_wrapper.params.yaml')

def generate_launch_description():

  ############# BNO055

  # Publishes:
  #   /bno055/imu: sensor_msgs/msg/Imu
  #   /bno055/calib_status: std_msgs/msg/String
  #   /bno055/imu_raw: sensor_msgs/msg/Imu
  #   /bno055/mag: sensor_msgs/msg/MagneticField
  #   /bno055/temp: sensor_msgs/msg/Temperature
  # Service Servers:
  #   /bno055/calibration_request: example_interfaces/srv/Trigger
  # Subscribes to: None
  # Service Clients: None
  # Action Servers: None
  # Action Clients: None

  bno055_driver_node = Node(
      package='bno055',
      executable='bno055',
      output='screen',
      parameters=[BNO055_DRIVER_PARAM_FILE]    
  )

  # Subscribes to:
  #  /bno055/imu_raw
  bno055_wrapper_node = Node(
          name='bno055_driver_wrapper_node',
          package='bno055_ros2_driver_wrapper',
          executable='bno055_driver_wrapper_node',
          output='screen',
          namespace='bno055',          
          parameters=[BNO055_WRAPPER_PARAM_FILE],
      )

  ############# LIDAR
  # Publishes:
  #   /lidar/points_raw: sensor_msgs/msg/LaserScan
  # Service Servers:
  #   /lidar/start_motor: std_srvs/srv/Empty
  #   /lidar/stop_motor: std_srvs/srv/Empty
  # Service Clients:
  # Action Servers:
  # Action Clients:

  sllidar_driver_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='lidar',
        parameters=[SLLIDAR_DRIVER_PARAM_FILE],
        output='screen',
        remappings=[
            ('scan', 'points_raw')
        ]
    )


  # Subscribes to:
  #   /lidar/points_raw                    (sensor_msgs::msg::PointCloud2)
  sllidar_ros2_wrapper_node = Node(
          name='sllidar_driver_wrapper_node',
          package='sllidar_ros2_driver_wrapper',
          executable='sllidar_driver_wrapper_node',
          namespace='lidar',
          parameters=[SLLIDAR_WRAPPER_PARAM_FILE],
          output='screen',
      )

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
        PushRosNamespace('hardware_interface'),
        joy_driver_node,
        joy_wrapper_node,
        vesc_driver_node,
        vesc_wrapper_node,
        bno055_driver_node,
        bno055_wrapper_node,
        sllidar_driver_node,
        sllidar_ros2_wrapper_node
    ])