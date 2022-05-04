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

  joy_sub_ = nh_->create_subscription<sensor_msgs::msg::Joy>(
    "joy",
    rclcpp::QoS(10),
    std::bind(&Buttons::joy_callback, this, std::placeholders::_1));

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
    buttons_.at(HMI_1 + i).set_state(
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

void Buttons::joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  buttons_.at(CONTROLLER_A).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[0]));
  buttons_.at(CONTROLLER_B).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[1]));
  buttons_.at(CONTROLLER_X).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[3]));
  buttons_.at(CONTROLLER_Y).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[2]));

  buttons_.at(CONTROLLER_UP).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->axes[7] == 1.0f));
  buttons_.at(CONTROLLER_DOWN).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->axes[7] == -1.0f));
  buttons_.at(CONTROLLER_LEFT).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->axes[6] == 1.0f));
  buttons_.at(CONTROLLER_RIGHT).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->axes[6] == -1.0f));

  buttons_.at(CONTROLLER_L1).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[4]));
  buttons_.at(CONTROLLER_L2).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[6]));
  buttons_.at(CONTROLLER_L3).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[11]));
  buttons_.at(CONTROLLER_R1).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[5]));
  buttons_.at(CONTROLLER_R2).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[7]));
  buttons_.at(CONTROLLER_R3).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[12]));

  buttons_.at(CONTROLLER_SHARE).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[8]));
  buttons_.at(CONTROLLER_OPTIONS).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[9]));
  buttons_.at(CONTROLLER_HOME).set_state(
    static_cast<Turtlebot4ButtonState>(joy_msg->buttons[10]));
}
