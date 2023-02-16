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
project(bno055_ros2_driver_wrapper)

# Get dependencies
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Include
include_directories(include)

# Build and register the composable node
ament_auto_add_library(bno055_driver_wrapper_composable_node
  SHARED src/DriverWrapper.cpp)
rclcpp_components_register_nodes(bno055_driver_wrapper_composable_node
  "bno055_ros2_driver_wrapper::DriverWrapper")

# Build regular node and link against the composable node
ament_auto_add_executable(bno055_driver_wrapper_node src/node.cpp)
target_link_libraries(bno055_driver_wrapper_node
  bno055_driver_wrapper_composable_node)

# Install parameters and launch files
ament_auto_package(INSTALL_TO_SHARE config launch)