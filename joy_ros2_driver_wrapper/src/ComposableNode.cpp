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

// STL includes
#include <memory>
#include <unordered_set>

// This project includes
#include "joy_ros2_driver_wrapper/ComposableNode.hpp"

namespace joy_ros2_driver_wrapper
{   
    ComposableNode::ComposableNode(const rclcpp::NodeOptions &options)
        : CarmaLifecycleNode(options)
    {
        config_ = ComposableNodeConfig();
        config_.joy_timeout = declare_parameter<double>("joy_timeout", config_.joy_timeout);
        config_.timer_callback = declare_parameter<int>("timer_callback", config_.timer_callback);
        config_.steering_axis = declare_parameter<int>("steering_axis", config_.steering_axis);
        config_.steering_scale = declare_parameter<double>("steering_scale", config_.steering_scale);
        config_.steering_offset = declare_parameter<double>("steering_offset", config_.steering_offset);
        config_.speed_axis = declare_parameter<int>("speed_axis", config_.speed_axis);
        config_.speed_scale = declare_parameter<double>("speed_scale", config_.speed_scale);
        config_.speed_offset = declare_parameter<double>("speed_offset", config_.speed_offset);
        config_.enable_button = declare_parameter<int>("enable_button", config_.enable_button);
        config_.enable_debounce = declare_parameter<double>("enable_debounce", config_.enable_debounce);
    }

    void ComposableNode::joy_callback(
        [[maybe_unused]] const sensor_msgs::msg::Joy::UniquePtr msg)
    {
        // Save the last time a joy event was published
        last_joy_time_ = this->now();

        // If the button is pushed (it moves from a value of 0 to 1)
        if (msg->buttons[config_.enable_button] != 0) {
          
          // Check the last time it was depressed, and debounce
          const rclcpp::Duration time_since_last_event = this->now() - last_enabled_time_;
          if (time_since_last_event.seconds() > config_.enable_debounce) {
            
            // Toggle the enabled state
            enabled_ = !enabled_;
            
            // Publish the new state
            std_msgs::msg::Bool engage_msg;
            engage_msg.data = enabled_;
            engage_pub_->publish(engage_msg);
          }
          
          // Mark this as the latest update time (for future debounce)
          last_enabled_time_ = this->now();
        }

        // Package up steering and speed into a vehicle command message
        autoware_msgs::msg::VehicleCmd vehicle_cmd_msg;
        vehicle_cmd_msg.ctrl_cmd.linear_velocity = 
          msg->axes[config_.speed_axis] * config_.speed_scale + config_.speed_offset;
        vehicle_cmd_msg.ctrl_cmd.steering_angle = 
          msg->axes[config_.steering_axis] * config_.steering_scale + config_.steering_offset;
        vehicle_cmd_pub_->publish(vehicle_cmd_msg);
    }
    
    carma_ros2_utils::CallbackReturn ComposableNode::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Joy driver wrapper trying to configure");

        // Create initial config
        config_ = ComposableNodeConfig();

        // Load Parameters
        get_parameter<double>("joy_timeout", config_.joy_timeout);
        get_parameter<int>("timer_callback", config_.timer_callback);
        get_parameter<int>("steering_axis", config_.steering_axis);
        get_parameter<double>("steering_scale", config_.steering_scale);
        get_parameter<double>("steering_offset", config_.steering_offset);
        get_parameter<int>("speed_axis", config_.speed_axis);
        get_parameter<double>("speed_scale", config_.speed_scale);
        get_parameter<double>("speed_offset", config_.speed_offset);
        get_parameter<int>("enable_button", config_.enable_button);
        get_parameter<double>("enable_debounce", config_.enable_debounce);
        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config: " << config_);
        
        // Make sure joy time is initialized before the first callback that needs it happens.
        last_enabled_time_ = this->now();

        // Add subscribers for the imu
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("input/joy", 10,
            std::bind(&ComposableNode::joy_callback, this, std::placeholders::_1));
        
        // Add publishers for the control output
        vehicle_cmd_pub_ = create_publisher<autoware_msgs::msg::VehicleCmd>("controller/vehicle_cmd", 10);
        engage_pub_ = create_publisher<std_msgs::msg::Bool>("vehicle/engage", 10);

        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn ComposableNode::handle_on_activate(const rclcpp_lifecycle::State &){
        // Disengage by default
        std_msgs::msg::Bool engage_msg;
        engage_msg.data = enabled_;
        engage_pub_->publish(engage_msg);

        // Start s health timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(config_.timer_callback), 
          std::bind(&ComposableNode::timer_callback, this));
        last_joy_time_ = this->now();

        return CallbackReturn::SUCCESS;
    }

    void ComposableNode::timer_callback(){
        // rclcpp::Duration duration_joy = this->now() - last_joy_time_;
        // if (duration_joy.seconds() > config_.joy_timeout) {
        //     throw std::invalid_argument("Joy message wait timed out");
        // }
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(joy_ros2_driver_wrapper::ComposableNode)