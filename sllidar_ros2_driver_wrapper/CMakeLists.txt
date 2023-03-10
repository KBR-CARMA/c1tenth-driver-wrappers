# Copyright (C) 2021 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.5)
project(sllidar_ros2_driver_wrapper)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Includedirectories
include_directories(include)

# Build the composable node
ament_auto_add_library(sllidar_driver_wrapper_composable_node SHARED
  src/ComposableNode.cpp)

# Build the node
ament_auto_add_executable(sllidar_driver_wrapper_node
  src/node.cpp)

# Register component
rclcpp_components_register_nodes(sllidar_driver_wrapper_composable_node
  "sllidar_ros2_driver_wrapper::ComposableNode")

target_link_libraries(sllidar_driver_wrapper_node
  sllidar_driver_wrapper_composable_node)

# Build tool to convert a laser scan to a point cloud 2 message
ament_auto_add_executable(lidar_scan_to_point_cloud2
  tools/lidar_scan_to_point_cloud2.cpp)

# Install products other than libraries, headers and executables
ament_auto_package(INSTALL_TO_SHARE config launch)
