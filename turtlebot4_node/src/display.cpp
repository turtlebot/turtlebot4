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

#include "turtlebot4_node/display.hpp"

#include <string>
#include <vector>
#include <memory>
#include <utility>

using turtlebot4::Display;
using turtlebot4::Turtlebot4MenuEntry;

/**
 * @brief Display constructor
 * @input nh - Turtlebot4 Node Handle
 */
Display::Display(
  std::vector<Turtlebot4MenuEntry> entries,
  std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh),
  update_required_(true),
  menu_entries_(entries),
  menu_override_(false),
  scroll_position_(0),
  selected_line_(0),
  ip_(std::string(UNKNOWN_IP)),
  battery_percentage_(0)
{
  RCLCPP_INFO(nh_->get_logger(), "Init Display");

  display_pub_ =
    nh_->create_publisher<turtlebot4_msgs::msg::UserDisplay>(
    "hmi/display",
    rclcpp::SensorDataQoS());
  display_message_sub_ = nh_->create_subscription<std_msgs::msg::String>(
    "hmi/display/message",
    rclcpp::SensorDataQoS(),
    std::bind(&Display::display_message_callback, this, std::placeholders::_1));

  // Initialize menu entries
  set_menu_entries();
  visible_entries_ = get_visible_entries();
}

/**
 * @brief Set IP address
 * @input ip - IP address as std::string
 */
void Display::set_ip(std::string ip)
{
  if (ip_ != ip) {
    ip_ = ip;
    request_update();
  }
}

/**
 * @brief Set battery percentage
 * @input battery_state_msg - Battery state message from Create3
 */
void Display::set_battery(const sensor_msgs::msg::BatteryState::SharedPtr & battery_state_msg)
{
  if (battery_percentage_ != static_cast<int>(battery_state_msg->percentage * 100)) {
    battery_percentage_ = static_cast<int>(battery_state_msg->percentage * 100);
    request_update();
  }
}

void Display::scroll_down()
{
  if (menu_override_) {
    return;
  }
  // Last possible scroll position, last line selected
  if (static_cast<size_t>(scroll_position_ + DISPLAY_NUM_LINES) == menu_entries_.size() &&
    selected_line_ == DISPLAY_NUM_LINES - 1)
  {
    return;
  }

  if (selected_line_ == DISPLAY_NUM_LINES - 1) {
    if (menu_entries_.size() > static_cast<size_t>(scroll_position_ + DISPLAY_NUM_LINES)) {
      scroll_position_++;
    }
  } else {
    selected_line_++;
  }
  request_update();
}

void Display::scroll_up()
{
  if (menu_override_) {
    return;
  }
  // First scroll position, first line selected
  if (scroll_position_ == 0 && selected_line_ == 0) {
    return;
  }

  if (selected_line_ == 0) {
    scroll_position_--;
  } else {
    selected_line_--;
  }
  request_update();
}

void Display::select()
{
  if (menu_override_) {
    return;
  }

  visible_entries_[selected_line_].function_call();
  request_update();
}

void Display::back()
{
  if (menu_override_) {
    menu_override_ = false;
  } else {
    scroll_position_ = 0;
    selected_line_ = 0;
  }
  request_update();
}

/**
 * @brief Format and return default display message
 */
void Display::set_menu_entries()
{
  for (auto & entry : menu_entries_) {
    pad_line(entry.name_);
  }
}

void Display::pad_line(std::string & line)
{
  // Pad string
  if (line.length() < DISPLAY_CHAR_PER_LINE) {
    line.insert(line.length(), DISPLAY_CHAR_PER_LINE - line.length(), ' ');
  } else if (line.length() > DISPLAY_CHAR_PER_LINE) {
    // Remove excess characters
    line = line.substr(0, DISPLAY_CHAR_PER_LINE);
  }
}

std::vector<Turtlebot4MenuEntry> Display::get_visible_entries()
{
  std::vector<Turtlebot4MenuEntry>::const_iterator first = menu_entries_.begin() + scroll_position_;
  std::vector<Turtlebot4MenuEntry>::const_iterator last;

  if (menu_entries_.size() > static_cast<size_t>(scroll_position_ + DISPLAY_NUM_LINES)) {
    last = menu_entries_.begin() + scroll_position_ + DISPLAY_NUM_LINES;
  } else {
    last = menu_entries_.end();
  }
  return std::vector<Turtlebot4MenuEntry>(first, last);
}

void Display::show_message(std::vector<std::string> message)
{
  std::vector<std::string>::const_iterator first = message.begin();
  std::vector<std::string>::const_iterator last;

  if (message.size() > static_cast<size_t>(DISPLAY_NUM_LINES)) {
    last = message.begin() + DISPLAY_NUM_LINES;
  } else {
    last = message.end();
  }
  display_lines_ = std::vector<std::string>(first, last);

  for (auto & line : display_lines_) {
    pad_line(line);
  }

  menu_override_ = true;
  request_update();
}

void Display::show_message(std::string message)
{
  display_lines_ = std::vector<std::string>(DISPLAY_NUM_LINES);

  for (int i = 0; i < DISPLAY_NUM_LINES; i++) {
    if (message.length() < static_cast<size_t>(DISPLAY_CHAR_PER_LINE * i)) {
      display_lines_[i] = "";
    } else if (message.length() < static_cast<size_t>(DISPLAY_CHAR_PER_LINE * (i + 1))) {
      display_lines_[i] =
        message.substr(DISPLAY_CHAR_PER_LINE * i, message.length() - (DISPLAY_CHAR_PER_LINE * i));
    } else {
      display_lines_[i] = message.substr(DISPLAY_CHAR_PER_LINE * i, DISPLAY_CHAR_PER_LINE);
    }
  }

  for (auto & line : display_lines_) {
    pad_line(line);
  }

  menu_override_ = true;
  request_update();
}

/**
 * @brief Update display
 */
void Display::update()
{
  visible_entries_ = get_visible_entries();

  auto display_msg = std::make_unique<turtlebot4_msgs::msg::UserDisplay>();

  display_msg->ip = ip_;
  display_msg->battery = std::to_string(battery_percentage_);

  if (menu_override_) {
    display_msg->selected_entry = -1;
    for (size_t i = 0; i < display_lines_.size(); i++) {
      display_msg->entries[i] = display_lines_.at(i);
    }
  } else {
    display_msg->selected_entry = selected_line_;
    for (size_t i = 0; i < visible_entries_.size(); i++) {
      display_msg->entries[i] = visible_entries_.at(i).name_;
    }
  }

  display_pub_->publish(std::move(display_msg));
}

/**
 * @brief Spin Once
 */
void Display::spin_once()
{
  if (update_required_) {
    update();
    update_required_ = false;
  }
}

void Display::request_update()
{
  update_required_ = true;
}

void Display::display_message_callback(const std_msgs::msg::String::SharedPtr display_msg)
{
  show_message(display_msg->data);
}
