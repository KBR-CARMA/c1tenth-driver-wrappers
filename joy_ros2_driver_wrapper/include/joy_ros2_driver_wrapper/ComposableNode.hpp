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
#include <autoware_msgs/msg/vehicle_cmd.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>

// CARMA includes
#include <carma_msgs/msg/system_alert.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

// This project includes
#include "joy_ros2_driver_wrapper/ComposableNodeConfig.hpp"

namespace joy_ros2_driver_wrapper
{
  class ComposableNode : public carma_ros2_utils::CarmaLifecycleNode {
  public:

    ComposableNode() = delete;

    // Constructor
    explicit ComposableNode(const rclcpp::NodeOptions &options);

    // Default destructore
    ~ComposableNode() = default;    

  private:
    // Lifycle state machine callback
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &prev_state);

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Publishers
    rclcpp::Publisher<autoware_msgs::msg::VehicleCmd>::SharedPtr vehicle_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr engage_pub_;

    // Configuration
    ComposableNodeConfig config_;

    // Callbacks
    void joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg);
    void timer_callback();

    // Variables
    bool enabled_ = false;
    rclcpp::Time last_joy_time_;
    rclcpp::Time last_enabled_time_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

} // namespace joy_ros2_driver_wrapper
