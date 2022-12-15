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

#include "turtlebot4_node/leds.hpp"

#include <memory>

using turtlebot4::Leds;

Leds::Leds(
  std::shared_ptr<rclcpp::Node> & nh)
: nh_(nh)
{
  RCLCPP_INFO(nh_->get_logger(), "Leds Init");

  // Power
  leds_ = {
    {Turtlebot4LedEnum::POWER, std::make_shared<Turtlebot4Led>(Turtlebot4LedType::GREEN_ONLY)},
    {Turtlebot4LedEnum::MOTORS, std::make_shared<Turtlebot4Led>(Turtlebot4LedType::GREEN_ONLY)},
    {Turtlebot4LedEnum::COMMS, std::make_shared<Turtlebot4Led>(Turtlebot4LedType::GREEN_ONLY)},
    {Turtlebot4LedEnum::WIFI, std::make_shared<Turtlebot4Led>(Turtlebot4LedType::GREEN_ONLY)},
    {Turtlebot4LedEnum::BATTERY, std::make_shared<Turtlebot4Led>(Turtlebot4LedType::RED_GREEN)},
    {Turtlebot4LedEnum::USER_1, std::make_shared<Turtlebot4Led>(Turtlebot4LedType::GREEN_ONLY)},
    {Turtlebot4LedEnum::USER_2, std::make_shared<Turtlebot4Led>(Turtlebot4LedType::RED_GREEN)},
  };

  user_led_sub_ = nh_->create_subscription<turtlebot4_msgs::msg::UserLed>(
    "hmi/led",
    rclcpp::SensorDataQoS(),
    std::bind(&Leds::user_led_callback, this, std::placeholders::_1));

  leds_[Turtlebot4LedEnum::POWER]->create_publisher(nh_, "hmi/led/_power");
  leds_[Turtlebot4LedEnum::MOTORS]->create_publisher(nh_, "hmi/led/_motors");
  leds_[Turtlebot4LedEnum::COMMS]->create_publisher(nh_, "hmi/led/_comms");
  leds_[Turtlebot4LedEnum::WIFI]->create_publisher(nh_, "hmi/led/_wifi");
  leds_[Turtlebot4LedEnum::BATTERY]->create_publisher(nh_, "hmi/led/_battery");
  leds_[Turtlebot4LedEnum::USER_1]->create_publisher(nh_, "hmi/led/_user1");
  leds_[Turtlebot4LedEnum::USER_2]->create_publisher(nh_, "hmi/led/_user2");
}

void Leds::spin_once()
{
  for (uint8_t i = 0; i < Turtlebot4LedEnum::COUNT; i++) {
    leds_[static_cast<Turtlebot4LedEnum>(i)]->spin_once();
  }
}

void Leds::user_led_callback(const turtlebot4_msgs::msg::UserLed user_led_msg)
{
  Turtlebot4LedEnum led = user_led_msg.led == 0 ? USER_1 : USER_2;
  blink(
    led, user_led_msg.blink_period, user_led_msg.duty_cycle,
    static_cast<Turtlebot4LedColor>(user_led_msg.color));
}

void Leds::set_led(Turtlebot4LedEnum led, Turtlebot4LedColor color)
{
  if (color == Turtlebot4LedColor::OFF) {
    blink(led, 1000, 0.0, color);
  } else {
    blink(led, 1000, 1.0, color);
  }
}

void Leds::blink(
  Turtlebot4LedEnum led, uint32_t blink_period_ms, double duty_cycle,
  Turtlebot4LedColor color)
{
  // Invalid duty cycle
  if (duty_cycle > 1.0 || duty_cycle < 0.0) {
    RCLCPP_ERROR(nh_->get_logger(), "Invalid duty cycle %f", duty_cycle);
    return;
  }

  if (color > Turtlebot4LedColor::GREEN && leds_[led]->type_ == Turtlebot4LedType::GREEN_ONLY) {
    RCLCPP_ERROR(nh_->get_logger(), "Invalid color %d for led %d", color, led);
    return;
  }

  leds_[led]->on_period_ms_ = blink_period_ms * duty_cycle;
  leds_[led]->off_period_ms_ = blink_period_ms * (1 - duty_cycle);
  leds_[led]->blink_color_ = color;
}
