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

#ifndef TURTLEBOT4_NODE__LEDS_HPP_
#define TURTLEBOT4_NODE__LEDS_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"

#include "turtlebot4_msgs/msg/user_led.hpp"
#include "turtlebot4_node/utils.hpp"

namespace turtlebot4
{

struct Turtlebot4Led
{
  Turtlebot4LedType type_;
  uint32_t on_period_ms_, off_period_ms_;
  std::chrono::time_point<std::chrono::steady_clock> last_on_time_, last_off_time_;
  Turtlebot4LedColor current_color_, next_color_, blink_color_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr led_pub_;

  explicit Turtlebot4Led(Turtlebot4LedType type)
  : type_(type),
    on_period_ms_(0),
    off_period_ms_(1000),
    current_color_(Turtlebot4LedColor::OFF),
    next_color_(Turtlebot4LedColor::OFF),
    blink_color_(Turtlebot4LedColor::OFF)
  {}

  void create_publisher(rclcpp::Node::SharedPtr nh, std::string topic)
  {
    led_pub_ = nh->create_publisher<std_msgs::msg::Int32>(topic, rclcpp::QoS(rclcpp::KeepLast(10)));
  }

  void spin_once()
  {
    switch (current_color_) {
      case Turtlebot4LedColor::OFF:
        {
          // Duty cycle > 0
          if (on_period_ms_ > 0) {
            // Time to turn on
            if (std::chrono::steady_clock::now() >
              last_off_time_ + std::chrono::milliseconds(off_period_ms_))
            {
              last_on_time_ = std::chrono::steady_clock::now();
              current_color_ = blink_color_;
            }
          }
          break;
        }

      case Turtlebot4LedColor::GREEN:
      case Turtlebot4LedColor::RED:
      case Turtlebot4LedColor::YELLOW:
        {
          // Duty cycle < 1.0 or blink color is OFF
          if (off_period_ms_ > 0 || blink_color_ != current_color_) {
            // Time to blink off
            if (std::chrono::steady_clock::now() >
              last_on_time_ + std::chrono::milliseconds(on_period_ms_))
            {
              last_off_time_ = std::chrono::steady_clock::now();
              current_color_ = Turtlebot4LedColor::OFF;
            }
          }
          break;
        }

      default:
        {
          return;
        }
    }

    auto msg = std_msgs::msg::Int32();
    msg.data = static_cast<int32_t>(current_color_);
    led_pub_->publish(msg);
  }
};

class Leds
{
public:
  Leds(
    std::shared_ptr<rclcpp::Node> & nh);

  void spin_once();
  void set_led(Turtlebot4LedEnum led, Turtlebot4LedColor color);
  void blink(
    Turtlebot4LedEnum led, uint32_t blink_period_ms, double duty_cycle,
    Turtlebot4LedColor color);

private:
  void user_led_callback(const turtlebot4_msgs::msg::UserLed user_led_msg);

  std::shared_ptr<rclcpp::Node> nh_;

  rclcpp::Subscription<turtlebot4_msgs::msg::UserLed>::SharedPtr user_led_sub_;

  std::map<Turtlebot4LedEnum, std::shared_ptr<Turtlebot4Led>> leds_;
};

}  // namespace turtlebot4

#endif  // TURTLEBOT4_NODE__LEDS_HPP_
