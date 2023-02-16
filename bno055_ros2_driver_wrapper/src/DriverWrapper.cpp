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

// ROS2 Messages
#include <sensor_msgs/msg/imu.hpp>

// This project includes
#include "bno055_ros2_driver_wrapper/DriverWrapper.hpp"

using std_msec = std::chrono::milliseconds;

namespace bno055_ros2_driver_wrapper
{   
    DriverWrapper::DriverWrapper(const rclcpp::NodeOptions &options)
        : CarmaLifecycleNode(options)
    {
        config_ = DriverWrapperConfig();
        config_.imu_timeout = this->declare_parameter<double>("imu_timeout", config_.imu_timeout);
        config_.timer_callback = this->declare_parameter<int>("timer_callback", config_.timer_callback);
    }
    
    void DriverWrapper::imu_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
    {
        last_imu_msg_ = this->now();
    }
    
    carma_ros2_utils::CallbackReturn DriverWrapper::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "BNO055 Driver wrapper trying to configure");

        // Create initial config
        config_ = DriverWrapperConfig();

        // Load Parameters
        this->get_parameter<double>("imu_timeout", config_.imu_timeout);
        this->get_parameter<int>("timer_callback", config_.timer_callback);

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded config: " << config_);
        
        // Add subscribers for the imu
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_raw", 5,
            std::bind(&DriverWrapper::imu_callback, this, std::placeholders::_1));
        
        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn DriverWrapper::handle_on_activate(const rclcpp_lifecycle::State &){

        timer_ = this->create_wall_timer(std::chrono::milliseconds(config_.timer_callback), 
        std::bind(&DriverWrapper::timerCallback, this));

        last_imu_msg_ = this->now();

        return CallbackReturn::SUCCESS;
    }

    void DriverWrapper::timerCallback(){
        rclcpp::Duration duration_imu = this->now() - last_imu_msg_;
        if (duration_imu.seconds() > config_.imu_timeout) {
            throw std::invalid_argument("IMU message wait timed out");
        }
    }

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(bno055_ros2_driver_wrapper::DriverWrapper)