#pragma once

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

#include <iostream>
#include <vector>

namespace joy_ros2_driver_wrapper
{
     /**
  * \brief Stuct containing the algorithm configuration values for the ComposableNode
  */

    struct ComposableNodeConfig
    {
        double joy_timeout = 0.25;
        int timer_callback = 500;
        int steering_axis = 2;
        double steering_scale = 0.48;
        double steering_offset = 0.0;
        int speed_axis = 1;
        double speed_scale = 2.0;
        double speed_offset = 0.0;
        int enable_button = 5;
        double enable_debounce = 0.2;

        // Stream operator for this config
        friend std::ostream &operator<<(std::ostream &output, const ComposableNodeConfig &c)
        {
            output  << "ComposableNodeConfig { " << std::endl
                    << "joy_timeout: " << c.joy_timeout << std::endl
                    << "timer_callback: " << c.timer_callback << std::endl
                    << "steering_axis: " << c.steering_axis << std::endl
                    << "steering_scale: " << c.steering_scale << std::endl
                    << "steering_offset: " << c.steering_offset << std::endl
                    << "speed_axis: " << c.speed_axis << std::endl
                    << "speed_scale: " << c.speed_scale << std::endl
                    << "speed_offset: " << c.speed_offset << std::endl
                    << "enable_button: " << c.enable_button << std::endl
                    << "enable_debounce: " << c.enable_debounce << std::endl;
            return output;
        }

    };
}  //namespace joy_ros2_driver_wrapper