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

#ifndef TURTLEBOT4_NODE__TURTLEBOT4_HPP_
#define TURTLEBOT4_NODE__TURTLEBOT4_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "turtlebot4_node/action.hpp"
#include "turtlebot4_node/service.hpp"
#include "turtlebot4_node/display.hpp"
#include "turtlebot4_node/buttons.hpp"
#include "turtlebot4_node/leds.hpp"
#include "turtlebot4_node/utils.hpp"

#include "irobot_create_msgs/msg/wheel_status.hpp"
#include "irobot_create_msgs/msg/lightring_leds.hpp"
#include "irobot_create_msgs/msg/dock_status.hpp"
#include "irobot_create_msgs/action/undock.hpp"
#include "irobot_create_msgs/action/dock.hpp"
#include "irobot_create_msgs/action/wall_follow.hpp"
#include "irobot_create_msgs/action/led_animation.hpp"
#include "irobot_create_msgs/srv/e_stop.hpp"
#include "irobot_create_msgs/srv/robot_power.hpp"


/** Supported functions
 * Dock
 * Undock
 * Follow
 * Power off
 * EStop
 */

namespace turtlebot4
{

// Timer Periods
static constexpr auto BUTTONS_TIMER_PERIOD = 10;
static constexpr auto COMMS_TIMER_PERIOD = 30000;
static constexpr auto DISPLAY_TIMER_PERIOD = 50;
static constexpr auto LEDS_TIMER_PERIOD = 50;
static constexpr auto POWER_OFF_TIMER_PERIOD = 60000;
static constexpr auto WIFI_TIMER_PERIOD = 5000;

class Turtlebot4 : public rclcpp::Node
{
public:
  // Type alias for actions and services
  using Dock = irobot_create_msgs::action::Dock;
  using Undock = irobot_create_msgs::action::Undock;
  using WallFollow = irobot_create_msgs::action::WallFollow;
  using LedAnimation = irobot_create_msgs::action::LedAnimation;
  using EStop = irobot_create_msgs::srv::EStop;
  using Power = irobot_create_msgs::srv::RobotPower;
  using EmptySrv = std_srvs::srv::Empty;
  using TriggerSrv = std_srvs::srv::Trigger;

  // Constructor and Destructor
  Turtlebot4();
  virtual ~Turtlebot4() {}

private:
  void run();

  // Subscription callbacks
  void battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr battery_state_msg);
  void dock_status_callback(
    const irobot_create_msgs::msg::DockStatus::SharedPtr dock_status_msg);
  void wheel_status_callback(
    const irobot_create_msgs::msg::WheelStatus::SharedPtr wheel_status_msg);
  void joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr joy_msg);

  // Function callbacks
  void dock_function_callback();
  void undock_function_callback();
  void wall_follow_left_function_callback();
  void wall_follow_right_function_callback();
  void estop_function_callback();
  void power_function_callback();
  void rplidar_start_function_callback();
  void rplidar_stop_function_callback();
  void oakd_start_function_callback();
  void oakd_stop_function_callback();
  void scroll_up_function_callback();
  void scroll_down_function_callback();
  void select_function_callback();
  void back_function_callback();
  void help_function_callback();
  void unused_function_callback();
  void function_call_callback(std::string function_name);

  void add_button_function_callbacks();
  void add_menu_function_callbacks();

  void low_battery_animation();

  // Run display timer
  void display_timer(const std::chrono::milliseconds timeout);

  // Run buttons timer
  void buttons_timer(const std::chrono::milliseconds timeout);

  // Run leds timer
  void leds_timer(const std::chrono::milliseconds timeout);

  // Run wifi timer
  void wifi_timer(const std::chrono::milliseconds timeout);

  // Run comms timer
  void comms_timer(const std::chrono::milliseconds timeout);

  // Run power off timer
  void power_off_timer(const std::chrono::milliseconds timeout);

  // IP
  std::string get_ip();
  std::string wifi_interface_;

  // Node
  rclcpp::Node::SharedPtr node_handle_;

  // Turtlebot4 Functions
  std::vector<Turtlebot4Button> turtlebot4_buttons_;
  std::vector<Turtlebot4MenuEntry> turtlebot4_menu_entries_;
  std::map<std::string, turtlebot4_function_callback_t> function_callbacks_;
  std::map<Turtlebot4ButtonEnum, std::string> button_parameters_;

  // Display
  std::unique_ptr<Display> display_;

  // Buttons
  std::unique_ptr<Buttons> buttons_;

  // Leds
  std::unique_ptr<Leds> leds_;

  // Actions
  std::unique_ptr<Turtlebot4Action<Dock>> dock_client_;
  std::unique_ptr<Turtlebot4Action<Undock>> undock_client_;
  std::unique_ptr<Turtlebot4Action<WallFollow>> wall_follow_client_;
  std::unique_ptr<Turtlebot4Action<LedAnimation>> led_animation_client_;

  // Services
  std::unique_ptr<Turtlebot4Service<EStop>> estop_client_;
  std::unique_ptr<Turtlebot4Service<Power>> power_client_;
  std::unique_ptr<Turtlebot4EmptyService<EmptySrv>> rplidar_start_client_;
  std::unique_ptr<Turtlebot4EmptyService<EmptySrv>> rplidar_stop_client_;
  std::unique_ptr<Turtlebot4Service<TriggerSrv>> oakd_start_client_;
  std::unique_ptr<Turtlebot4Service<TriggerSrv>> oakd_stop_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr display_timer_;
  rclcpp::TimerBase::SharedPtr buttons_timer_;
  rclcpp::TimerBase::SharedPtr leds_timer_;
  rclcpp::TimerBase::SharedPtr wifi_timer_;
  rclcpp::TimerBase::SharedPtr comms_timer_;
  rclcpp::TimerBase::SharedPtr power_off_timer_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::DockStatus>::SharedPtr dock_status_sub_;
  rclcpp::Subscription<irobot_create_msgs::msg::WheelStatus>::SharedPtr wheel_status_sub_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ip_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr function_call_pub_;

  // Store current wheels state
  bool wheels_enabled_;

  // Store current dock state
  bool is_docked_;

  // Store power saver mode
  bool power_saver_;

  // Turtlebot4 Model
  Turtlebot4Model model_;
};

}  // namespace turtlebot4

#endif  // TURTLEBOT4_NODE__TURTLEBOT4_HPP_
