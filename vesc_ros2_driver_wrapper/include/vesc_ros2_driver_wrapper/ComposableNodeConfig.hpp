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

namespace vesc_ros2_driver_wrapper
{
     /**
  * \brief Stuct containing the algorithm configuration values for the ComposableNode
  */

    struct ComposableNodeConfig
    {
        double vesc_state_timeout = 0.25;
        double vesc_servo_timeout = 0.25;
        int timer_callback = 500;
        double wheelbase = 0.3175;
        double speed_to_erpm_gain = -4221.0;
        double speed_to_erpm_offset = 0.0;
        double steering_to_servo_gain = -0.9135;
        double steering_to_servo_offset = 0.5304;
        double max_erpm_positive_delta = 500.0;
        double max_erpm_negative_delta = 1000.0;

        // Stream operator for this config
        friend std::ostream &operator<<(std::ostream &output, const ComposableNodeConfig &c)
        {
            output  << "ComposableNodeConfig { " << std::endl
                    << "vesc_state_timeout: " << c.vesc_state_timeout << std::endl
                    << "vesc_servo_timeout: " << c.vesc_servo_timeout << std::endl
                    << "timer_callback: " << c.timer_callback << std::endl
                    << "wheelbase: " << c.wheelbase << std::endl
                    << "speed_to_erpm_gain: " << c.speed_to_erpm_gain << std::endl
                    << "speed_to_erpm_offset: " << c.speed_to_erpm_offset << std::endl
                    << "steering_to_servo_gain: " << c.steering_to_servo_gain << std::endl
                    << "steering_to_servo_offset: " << c.steering_to_servo_offset << std::endl                    
                    << "max_erpm_positive_delta: " << c.max_erpm_positive_delta << std::endl
                    << "max_erpm_negative_delta: " << c.max_erpm_negative_delta << std::endl;
            return output;
        }

    };
}  //namespace vesc_ros2_driver_wrapper