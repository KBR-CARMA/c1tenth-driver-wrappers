#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# Source the CARMA and ROS2 toolchains.
source /opt/carma/install_ros2/setup.bash

# Change to our ROS2 workspace
cd /home/carma

# Install all required dependencies for the source code we pulled.
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build everything we need for our drivers
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to bno055_ros2_driver_wrapper \
                   joy_ros2_driver_wrapper \
                   sllidar_ros2_driver_wrapper \
                   vesc_ros2_driver_wrapper
