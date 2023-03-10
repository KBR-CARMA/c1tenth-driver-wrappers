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

#include <memory>
#include "sllidar_ros2_driver_wrapper/ComposableNode.hpp"

namespace sllidar_ros2_driver_wrapper
{
    ComposableNode::ComposableNode(const rclcpp::NodeOptions &options)
        : CarmaLifecycleNode(options)
    {
        // Init config
        config_ = ComposableNodeConfig();
        config_.point_cloud_timeout = this->declare_parameter<double>("point_cloud_timeout", config_.point_cloud_timeout);
        config_.timer_callback = this->declare_parameter<int>("timer_callback", config_.timer_callback);
    }

    void ComposableNode::point_cloud_callback(
        [[maybe_unused]] const sensor_msgs::msg::PointCloud2::UniquePtr msg)
    {
        last_update_time_ = this->now();
    }

    carma_ros2_utils::CallbackReturn ComposableNode::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Sllidar Driver wrapper trying to configure");

        // Load config
        config_ = ComposableNodeConfig();
        this->get_parameter<double>("point_cloud_timeout", config_.point_cloud_timeout);
        this->get_parameter<int>("timer_callback", config_.timer_callback);

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config: " << config_);

        // Add subscriber(s)
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("lidar/points_raw", 1,
            std::bind(&ComposableNode::point_cloud_callback, this, std::placeholders::_1));
        
        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn ComposableNode::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
    {
        // Timer setup
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
        std::bind(&ComposableNode::timer_callback, this));

        //Initialize timeout check
        last_update_time_ = this->now();
        
        return CallbackReturn::SUCCESS;
    }

    void ComposableNode::timer_callback(){
        // rclcpp::Duration duration_since_last_update = this->now() - last_update_time_;
        // if(duration_since_last_update.seconds() > config_.point_cloud_timeout){
        //     throw std::invalid_argument("Point Cloud wait timed out");
        // } 
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(sllidar_ros2_driver_wrapper::ComposableNode)
