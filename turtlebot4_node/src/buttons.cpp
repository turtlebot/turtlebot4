/*
 * Copyright 2021 Clearpath Robotics, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @author Roni Kreinin (rkreinin@clearpathrobotics.com)
 */

#include "turtlebot4_node/buttons.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <utility>

using turtlebot4::Buttons;

Buttons::Buttons(
  Turtlebot4Model model,
  std::vector<Turtlebot4Button> buttons,
  std::shared_ptr<rclcpp::Node> & nh)
: model_(model),
  buttons_(buttons),
  nh_(nh)
{
  RCLCPP_INFO(nh_->get_logger(), "Buttons Init");

  create3_buttons_sub_ = nh_->create_subscription<irobot_create_msgs::msg::InterfaceButtons>(
    "interface_buttons",
    rclcpp::SensorDataQoS(),
    std::bind(&Buttons::create3_buttons_callback, this, std::placeholders::_1));

  if (model_ == Turtlebot4Model::STANDARD) {
    hmi_buttons_sub_ = nh_->create_subscription<turtlebot4_msgs::msg::UserButton>(
      "hmi/buttons",
      rclcpp::SensorDataQoS(),
      std::bind(&Buttons::hmi_buttons_callback, this, std::placeholders::_1));
  }
}

/**
 * @brief Poll buttons, call callbacks if pressed
 */
void Buttons::spin_once()
{
  // Spin buttons
  for (size_t i = 0; i < buttons_.size(); i++) {
    buttons_.at(i).spin_once();
  }
}

void Buttons::hmi_buttons_callback(
  const turtlebot4_msgs::msg::UserButton::SharedPtr hmi_buttons_msg)
{
  for (uint8_t i = 0; i < HMI_BUTTON_COUNT; i++) {
    buttons_.at(CREATE3_BUTTON_COUNT + i).set_state(
      static_cast<Turtlebot4ButtonState>(hmi_buttons_msg->button[i]));
  }
}

void Buttons::create3_buttons_callback(
  const irobot_create_msgs::msg::InterfaceButtons::SharedPtr create3_buttons_msg)
{
  buttons_.at(CREATE3_1).set_state(
    static_cast<Turtlebot4ButtonState>(create3_buttons_msg->button_1.
    is_pressed));
  buttons_.at(CREATE3_POWER).set_state(
    static_cast<Turtlebot4ButtonState>(create3_buttons_msg->
    button_power.is_pressed));
  buttons_.at(CREATE3_2).set_state(
    static_cast<Turtlebot4ButtonState>(create3_buttons_msg->button_2.
    is_pressed));
}
