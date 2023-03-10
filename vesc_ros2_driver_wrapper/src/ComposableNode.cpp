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
#include "vesc_ros2_driver_wrapper/ComposableNode.hpp"

using std_msec = std::chrono::milliseconds;

namespace vesc_ros2_driver_wrapper
{   
    ComposableNode::ComposableNode(const rclcpp::NodeOptions &options)
        : CarmaLifecycleNode(options)
    {
        config_ = ComposableNodeConfig();
        config_.vesc_state_timeout = declare_parameter<double>(
            "vesc_state_timeout", config_.vesc_state_timeout);
        config_.vesc_servo_timeout = declare_parameter<double>(
            "vesc_servo_timeout", config_.vesc_servo_timeout);
        config_.timer_callback = declare_parameter<int>(
            "timer_callback", config_.timer_callback);
        config_.wheelbase = declare_parameter<double>(
            "wheelbase", config_.wheelbase);
        config_.speed_to_erpm_gain = declare_parameter<double>(
            "speed_to_erpm_gain", config_.speed_to_erpm_gain);
        config_.speed_to_erpm_offset = declare_parameter<double>(
            "speed_to_erpm_offset", config_.speed_to_erpm_offset);
        config_.steering_to_servo_gain = declare_parameter<double>(
            "steering_to_servo_gain", config_.steering_to_servo_gain);
        config_.steering_to_servo_offset = declare_parameter<double>(
            "steering_to_servo_offset", config_.steering_to_servo_offset);
        config_.max_erpm_positive_delta = declare_parameter<double>(
            "max_erpm_positive_delta", config_.max_erpm_positive_delta);
        config_.max_erpm_negative_delta = declare_parameter<double>(
            "max_erpm_negative_delta", config_.max_erpm_negative_delta);
    }

    void ComposableNode::vesc_state_callback(
        const vesc_msgs::msg::VescStateStamped::UniquePtr msg)
    {
        // For use in the next time tick
        last_vesc_state_time_ = this->now();

        // To publish a vehicle state we need to have servo state
        if (last_vesc_servo_msg_) {
            
            // Calculate the speed (in meters per second)
            double speed_m_per_s =
                (-msg->state.speed - config_.speed_to_erpm_offset) / config_.speed_to_erpm_gain;
            if (std::fabs(speed_m_per_s) < 0.05) {
                speed_m_per_s = 0.0;
            }
            
            // Calculate the steering angle in radians
            double steering_rads =
                (last_vesc_servo_msg_->data - config_.steering_to_servo_offset) / config_.steering_to_servo_gain;
            
            // Package up and send a vehicle state message to autoware
            autoware_msgs::msg::VehicleStatus vehicle_status;
            vehicle_status.header = msg->header;
            vehicle_status.speed = speed_m_per_s * 3.6;     // m/s -> km/h
            vehicle_status.angle = steering_rads;           // radians
            vehicle_status_pub_->publish(vehicle_status);

            // Package up and send odometry
            geometry_msgs::msg::TwistStamped twist;
            twist.header = msg->header;
            twist.twist.linear.x = speed_m_per_s;
            twist.twist.angular.z = speed_m_per_s * std::tan(steering_rads) / config_.wheelbase;
            current_twist_pub_->publish(twist);
        }
    }

    void ComposableNode::vesc_servo_callback(
        const std_msgs::msg::Float64::UniquePtr msg)
    {
        last_vesc_servo_time_ = this->now();
        last_vesc_servo_msg_ = *msg;
    }

    void ComposableNode::vehicle_cmd_callback(
        const autoware_msgs::msg::VehicleCmd::UniquePtr msg)
    {
        if (this->enabled_) {
            
            // Extract fields of interest
            double speed_m_per_s = msg->ctrl_cmd.linear_velocity;
            double steering_rads = msg->ctrl_cmd.steering_angle;

            // Persistent previous erpm value
            static double prev_erpm_value = 0.0;

            // Handle erpm
            std_msgs::msg::Float64 erpm_msg;
            if (speed_m_per_s == 0.0) {
                erpm_msg.data = 0.0;
            } else {
                erpm_msg.data = config_.speed_to_erpm_gain * speed_m_per_s + config_.speed_to_erpm_offset;
                erpm_msg.data = std::clamp(erpm_msg.data, 
                    prev_erpm_value - config_.max_erpm_negative_delta,
                    prev_erpm_value + config_.max_erpm_positive_delta);
            }
            prev_erpm_value = erpm_msg.data;
            erpm_pub_->publish(erpm_msg);

            // Handle steering angle
            std_msgs::msg::Float64 servo_msg;
            servo_msg.data = config_.steering_to_servo_gain * steering_rads + config_.steering_to_servo_offset;
            servo_pub_->publish(servo_msg);
        }
    }

    void ComposableNode::engage_callback(const std_msgs::msg::Bool::UniquePtr msg) {
        enabled_ = msg->data;
    }

    carma_ros2_utils::CallbackReturn ComposableNode::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "VESC driver wrapper trying to configure");

        // Create initial config
        config_ = ComposableNodeConfig();

        // Load Parameters
        get_parameter<double>("vesc_state_timeout", config_.vesc_state_timeout);
        get_parameter<double>("vesc_servo_timeout", config_.vesc_servo_timeout);
        get_parameter<int>("timer_callback", config_.timer_callback);
        get_parameter<double>("wheelbase", config_.wheelbase);
        get_parameter<double>("speed_to_erpm_gain", config_.speed_to_erpm_gain);
        get_parameter<double>("speed_to_erpm_offset", config_.speed_to_erpm_offset);
        get_parameter<double>("steering_to_servo_gain", config_.steering_to_servo_gain);
        get_parameter<double>("steering_to_servo_offset", config_.steering_to_servo_offset);
        get_parameter<double>("max_erpm_positive_delta", config_.max_erpm_positive_delta);
        get_parameter<double>("max_erpm_negative_delta", config_.max_erpm_negative_delta);
        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config: " << config_);
        
        // Publishers
        vehicle_status_pub_ = create_publisher<autoware_msgs::msg::VehicleStatus>("controller/vehicle_status", 10);
        current_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("controller/vehicle/twist", 10);
        erpm_pub_ = create_publisher<std_msgs::msg::Float64>("vesc/commands/motor/speed", 10);
        servo_pub_ = create_publisher<std_msgs::msg::Float64>("vesc/commands/servo/position", 10);

        // Subscribers
        vesc_state_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>("vesc/sensors/core", 10,
            std::bind(&ComposableNode::vesc_state_callback, this, std::placeholders::_1));
        vesc_servo_sub_ = create_subscription<std_msgs::msg::Float64>("vesc/sensors/servo_position_command", 10,
            std::bind(&ComposableNode::vesc_servo_callback, this, std::placeholders::_1));
        vehicle_cmd_sub_ = create_subscription<autoware_msgs::msg::VehicleCmd>("/vehicle_cmd", 10,
            std::bind(&ComposableNode::vehicle_cmd_callback, this, std::placeholders::_1));   
        engage_sub_ = create_subscription<std_msgs::msg::Bool>("/vehicle/engage", 10,
            std::bind(&ComposableNode::engage_callback, this, std::placeholders::_1));

        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn ComposableNode::handle_on_activate(const rclcpp_lifecycle::State &){
        timer_ = this->create_wall_timer(std::chrono::milliseconds(config_.timer_callback), 
            std::bind(&ComposableNode::timer_callback, this));
        last_vesc_state_time_ = this->now();
        last_vesc_servo_time_ = this->now();
        return CallbackReturn::SUCCESS;
    }

    void ComposableNode::timer_callback(){
        // rclcpp::Duration duration_vesc_state = this->now() - last_vesc_state_time_;
        // if (duration_vesc_state.seconds() > config_.vesc_state_timeout) {
        //     throw std::invalid_argument("VESC state message wait timed out");
        // }
        // rclcpp::Duration duration_vesc_servo = this->now() - last_vesc_servo_time_;
        // if (duration_vesc_servo.seconds() > config_.vesc_servo_timeout) {
        //     throw std::invalid_argument("VESC servo message wait timed out");
        // }
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ros2_driver_wrapper::ComposableNode)