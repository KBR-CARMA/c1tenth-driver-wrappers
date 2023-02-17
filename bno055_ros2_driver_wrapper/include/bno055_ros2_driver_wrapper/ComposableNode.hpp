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

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

// Message includes
#include <sensor_msgs/msg/imu.hpp>

// CARMA includes
#include <carma_msgs/msg/system_alert.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

// This project includes
#include "bno055_ros2_driver_wrapper/ComposableNodeConfig.hpp"

namespace bno055_ros2_driver_wrapper
{
  class ComposableNode : public carma_ros2_utils::CarmaLifecycleNode {
  public:

    ComposableNode() = delete;

    // Constructor
    explicit ComposableNode(const rclcpp::NodeOptions &options);

    // Default destructore
    ~ComposableNode() = default;    

    // Lifycle state machine callback
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &prev_state);

    // Add Subscribers
    rclcpp::Subscription<carma_msgs::msg::SystemAlert>::SharedPtr system_alert_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Wrapper configuration
    ComposableNodeConfig config_;

    // Callbacks
    void imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg);
    void timer_callback();

    rclcpp::Time last_imu_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

} // namespace bno055_ros2_driver_wrapper
