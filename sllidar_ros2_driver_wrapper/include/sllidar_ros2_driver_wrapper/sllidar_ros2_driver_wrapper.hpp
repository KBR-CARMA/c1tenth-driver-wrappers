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

#include "carma_ros2_utils/carma_lifecycle_node.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "sllidar_ros2_driver_wrapper/sllidar_ros2_driver_wrapper_config.hpp"

namespace sllidar_ros2_driver_wrapper
{
    class Node : public carma_ros2_utils::CarmaLifecycleNode
    {
        public:
        
        Node() = delete;

        /**
         * \brief Constructor. Set explicitly to support node composition.
         * 
         * \param options The node options to use for configuring this node
        */
       explicit Node(const rclcpp::NodeOptions &options);

       ~Node() = default;

        ////
        // Overrides
        ///
        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
        carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &prev_state);
        
        private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
        rclcpp::Time last_update_time_;
        double point_cloud_timeout_;

        void point_cloud_cb(const sensor_msgs::msg::PointCloud2::UniquePtr msg);
        void check_lidar_timeout();

        Config config_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
}
