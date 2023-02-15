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

// TODO: Resolve point cloud equivalent for IMU 
#include <memory>
#include "wrapper_template/wrapper_template.hpp"

namespace wrapper_template
{
    Node::Node(const rclcpp::NodeOptions &options)
        : CarmaLifecycleNode(options)
    {
        config_ = Config();
        //Load Parameters
        config_.point_cloud_timeout = this->declare_parameter<double>("wrapper_template_timeout", config_.point_cloud_timeout);
    }

    void Node::point_cloud_cb(const sensor_msgs::msg::PointCloud2::UniquePtr msg)
    {
        last_update_time_ = this->now();

    }

    carma_ros2_utils::CallbackReturn Node::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "wrapper template trying to configure");

        config_ = Config();
        //Load Parameters
        this->get_parameter<double>("wrapper_template_timeout", config_.point_cloud_timeout);

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config: " << config_);

        // Subscribe to the driver you are wrapping
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("lidar/points_raw", 1,
            std::bind(&Node::point_cloud_cb, this, std::placeholders::_1));
        
        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn Node::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
    {
        //Timer setup
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), 
        std::bind(&Node::check_lidar_timeout, this));

        //Initialize timeout check
        last_update_time_ = this->now();
        
        return CallbackReturn::SUCCESS;
    }

    void Node::check_lidar_timeout(){
        
        rclcpp::Duration duration_since_last_update = this->now() - last_update_time_;

        if(duration_since_last_update.seconds() > config_.point_cloud_timeout){
            throw std::invalid_argument("Point Cloud wait timed out");
        } 
    }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(wrapper_template::Node)