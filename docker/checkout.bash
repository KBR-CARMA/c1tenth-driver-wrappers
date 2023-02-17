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

# CARMA packages checkout script
# Optional argument to set the root checkout directory with no ending '/' default is '~'

set -exo pipefail

# Move to our source directory
cd /home/carma/src

# Check out a serial dependency
git clone --depth=1 https://github.com/KBR-CARMA/transport_drivers.git --branch c1tenth-develop

# Check out all the drivers
git clone --depth=1 https://github.com/KBR-CARMA/vesc.git --branch c1tenth-develop
git clone --depth=1 https://github.com/KBR-CARMA/sllidar_ros2.git --branch c1tenth-develop
git clone --depth=1 https://github.com/KBR-CARMA/dwm1001_ros2.git --branch c1tenth-develop

# TODO(asymingt) - fix the IMU problem from a recent commit.
git clone https://github.com/KBR-CARMA/bno055.git --branch c1tenth-develop
cd bno055
git checkout -b tmp b2eeb34413ddd77e57020b22875dbd3bedfacfb9