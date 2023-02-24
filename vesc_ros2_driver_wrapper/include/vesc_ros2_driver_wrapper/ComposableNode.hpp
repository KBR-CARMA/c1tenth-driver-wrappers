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
#include <autoware_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

// CARMA includes
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

// This project includes
#include "vesc_ros2_driver_wrapper/ComposableNodeConfig.hpp"

namespace vesc_ros2_driver_wrapper
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

    // Subscribers
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;    // From VESC
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vesc_servo_sub_;              // From VESC
    rclcpp::Subscription<autoware_msgs::msg::VehicleCmd>::SharedPtr vehicle_cmd_sub_;     // From Autoware
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr engage_sub_;                     // From Autoware

    // Publishers
    rclcpp::Publisher<autoware_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_pub_; // To Autoware
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr current_twist_pub_;    // To Autoware
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr erpm_pub_;                       // To VESC
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr servo_pub_;                      // To VESC
  
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;

    // Wrapper configuration
    ComposableNodeConfig config_;

    // Callbacks
    void vesc_state_callback(const vesc_msgs::msg::VescStateStamped::UniquePtr msg);
    void vesc_servo_callback(const std_msgs::msg::Float64::UniquePtr msg);
    void vehicle_cmd_callback(const autoware_msgs::msg::VehicleCmd::UniquePtr msg);
    void engage_callback(const std_msgs::msg::Bool::UniquePtr msg);
    void timer_callback();

    // INternal variables
    bool enabled_ = false;
    rclcpp::Time last_vesc_state_time_;
    rclcpp::Time last_vesc_servo_time_;
    std::optional<std_msgs::msg::Float64> last_vesc_servo_msg_;
  };

} // namespace vesc_ros2_driver_wrapper
