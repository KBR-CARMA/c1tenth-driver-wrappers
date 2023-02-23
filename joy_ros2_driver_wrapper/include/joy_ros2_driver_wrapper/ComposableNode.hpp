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
#include <sensor_msgs/msg/joy.hpp>


namespace joy_ros2_driver_wrapper {

/// A node which translates sensor_msgs/msg/Joy messages into messages compatible with the vehicle
/// interface. All participants use SensorDataQoS
class ComposableNode : public rclcpp::Node {
public:
  /// ROS 2 parameter constructor
  explicit ComposableNode(const rclcpp::NodeOptions & node_options);

private:
  std::unique_ptr<joystick_vehicle_interface::JoystickVehicleInterface> m_core;
  void init(
    const std::string & control_command,
    const std::string & state_command_topic,
    const std::string & joy_topic,
    const bool8_t & recordreplay_command_enabled,
    const AxisMap & axis_map,
    const AxisScaleMap & axis_scale_map,
    const AxisScaleMap & axis_offset_map,
    const ButtonMap & button_map);

  /// Callback for joystick subscription: compute control and state command and publish
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  template<typename T>
  void on_auto_cmd(const typename T::SharedPtr msg)
  {
    if (m_core->is_autonomous_mode_on()) {
      const auto pub_ctr_cmd = [this, &msg](auto && pub) -> void {
          using MessageT =
            typename std::decay_t<decltype(pub)>::element_type::MessageUniquePtr::element_type::
            SharedPtr;
          auto cmd = **reinterpret_cast<const MessageT *>(&msg);
          cmd.stamp = now();
          pub->publish(cmd);
        };
      mpark::visit(pub_ctr_cmd, m_cmd_pub);
    }
  }

  using HighLevelControl = autoware_auto_control_msgs::msg::HighLevelControlCommand;
  using BasicControl = autoware_auto_vehicle_msgs::msg::VehicleControlCommand;
  using RawControl = autoware_auto_vehicle_msgs::msg::RawControlCommand;
  template<typename T>
  using PubT = typename rclcpp::Publisher<T>::SharedPtr;
  template<typename T>
  using SubT = typename rclcpp::Subscription<T>::SharedPtr;

  using ControlPub = mpark::variant<PubT<RawControl>, PubT<BasicControl>, PubT<HighLevelControl>>;
  using ControlSub = mpark::variant<SubT<RawControl>, SubT<BasicControl>, SubT<HighLevelControl>>;

  ControlPub m_cmd_pub{};

  // input autonomous cmd for switching manual/auto mode at a button press
  // TODO(haoru): a seperate multiplexer will be developed in Autoware Universe
  ControlSub m_auto_cmd_sub{};

  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::VehicleStateCommand>::SharedPtr m_state_cmd_pub{};
  rclcpp::Publisher<autoware_auto_vehicle_msgs::msg::HeadlightsCommand>::SharedPtr m_headlights_cmd_pub{};
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_recordreplay_cmd_pub{};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub{nullptr};
};

} // namespace joy_ros2_driver_wrapper
