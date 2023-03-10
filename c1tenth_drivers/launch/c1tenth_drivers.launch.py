
import os

from ament_index_python import get_package_share_directory

import launch
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Include all individual package launch directories

IMU_LAUNCH_PATH = os.path.join(
    get_package_share_directory('bno055_ros2_driver_wrapper'),
    'launch', 'bno055_ros2_driver_wrapper.launch.py')

LIDAR_LAUNCH_PATH = os.path.join(
    get_package_share_directory('sllidar_ros2_driver_wrapper'),
    'launch', 'sllidar_ros2_driver_wrapper.launch.py')

JOY_LAUNCH_PATH = os.path.join(
    get_package_share_directory('joy_ros2_driver_wrapper'),
    'launch', 'joy_ros2_driver_wrapper.launch.py')

VESC_LAUNCH_PATH = os.path.join(
    get_package_share_directory('vesc_ros2_driver_wrapper'),
    'launch', 'vesc_ros2_driver_wrapper.launch.py')


def generate_launch_description():

    imu_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(IMU_LAUNCH_PATH),
        launch_arguments=[['composable', 'false']])

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(LIDAR_LAUNCH_PATH),
        launch_arguments=[['composable', 'false']])

    joy_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(JOY_LAUNCH_PATH),
        launch_arguments=[['composable', 'false']])

    vesc_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(VESC_LAUNCH_PATH),
        launch_arguments=[['composable', 'false']])


    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'address': 'localhost'},
            {'port': '9090'}
        ],
    )
    
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
        parameters=[
            {'topics_glob': ''},
            {'services_glob': ''},
            {'params_glob': ''},
            {'bson_only_mode', 'false'}
        ],
    )

    return launch.LaunchDescription([
        PushRosNamespace('hardware_interface'),
        imu_driver,
        lidar_driver,
        joy_driver,
        vesc_driver,
        rosbridge_websocket,
        rosapi_node
    ])
