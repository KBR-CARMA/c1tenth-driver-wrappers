import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
import os


template_wrapper_param_file = os.path.join(
  get_package_share_directory('wrapper_template'), 'config', 'wrapper_template.params.yaml')

def generate_launch_description():

  wrapper_template_node = Node(
          package='wrapper_template',
          executable='wrapper_template_node_exec',
          name='wrapper_template_node',
          namespace='template',
          parameters=[template_wrapper_param_file],
          output='screen'
      )

  return launch.LaunchDescription([
        wrapper_template_node
    ])