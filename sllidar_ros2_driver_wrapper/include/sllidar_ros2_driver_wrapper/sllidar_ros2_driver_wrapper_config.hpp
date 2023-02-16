/*
 * Copyright (C) 2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#pragma once

#include <iostream>
#include <vector>

namespace sllidar_ros2_driver_wrapper
{
    /**
  * \brief Stuct containing the algorithm configuration values for the VelodyneLidarDriverWrapper
  */

    struct Config
    {
        double point_cloud_timeout = 0.2;

        // Stream operator for this config
        friend std::ostream &operator<<(std::ostream &output, const Config &c)
        {
            output << "SllidarRos2DriverWrapper::Config { "<<std::endl
                   <<"point_cloud_timeout: "<<c.point_cloud_timeout<<std::endl;
            
            return output;
        }
    };
}//namespace sllidar_ros2_driver_wrapper
