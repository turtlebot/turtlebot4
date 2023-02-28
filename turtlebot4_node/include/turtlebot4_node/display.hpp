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

#ifndef TURTLEBOT4_NODE__DISPLAY_HPP_
#define TURTLEBOT4_NODE__DISPLAY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>


#include "turtlebot4_node/utils.hpp"
#include "turtlebot4_msgs/msg/user_display.hpp"


namespace turtlebot4
{

static constexpr auto DISPLAY_NUM_LINES = 5;
static constexpr auto DISPLAY_CHAR_PER_LINE = 18;
static constexpr auto DISPLAY_CHAR_PER_LINE_HEADER = 21;

struct Turtlebot4MenuEntry
{
  std::string name_, function_;
  turtlebot4_function_callback_t cb_;
  turtlebot4_function_call_callback_t function_call_cb_;

  explicit Turtlebot4MenuEntry(std::string name)
  : name_(name),
    function_(name)
  {}

  /**
   * @brief Call menu function
   *
   */
  void function_call()
  {
    if (function_call_cb_ != nullptr) {
      function_call_cb_(function_);
    }

    if (cb_ != nullptr) {
      cb_();
    }
  }
};

class Display
{
public:
  // Constructor and Destructor
  explicit Display(
    std::vector<Turtlebot4MenuEntry> entries,
    std::shared_ptr<rclcpp::Node> & nh);
  virtual ~Display() {}

  // Setters
  void set_battery(const sensor_msgs::msg::BatteryState::SharedPtr & battery_state_msg);
  void set_ip(std::string ip);

  // Menu Navigation
  void scroll_up();
  void scroll_down();
  void select();
  void back();
  void show_message(std::vector<std::string> message);
  void show_message(std::string message);

  // Spin Once
  void spin_once();

  // Request display update
  void request_update();

private:
  // Update display
  void update();
  void update_header();
  void set_menu_entries();
  void pad_line(std::string & line);
  std::vector<Turtlebot4MenuEntry> get_visible_entries();

  void display_message_callback(const std_msgs::msg::String::SharedPtr display_msg);

  // Node handle
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::Publisher<turtlebot4_msgs::msg::UserDisplay>::SharedPtr display_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr display_message_sub_;
  bool update_required_;

  // Menu
  std::vector<Turtlebot4MenuEntry> menu_entries_;
  std::vector<Turtlebot4MenuEntry> visible_entries_;
  std::vector<std::string> display_lines_;
  bool menu_override_;
  uint8_t scroll_position_;
  uint8_t selected_line_;

  // Header
  std::string ip_;
  int battery_percentage_;
  std::string header_;
};

}  // namespace turtlebot4

#endif  // TURTLEBOT4_NODE__DISPLAY_HPP_
