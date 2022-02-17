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

#ifndef TURTLEBOT4_NODE__UTILS_HPP_
#define TURTLEBOT4_NODE__UTILS_HPP_

#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <map>

namespace turtlebot4
{

#define CREATE3_BUTTON_COUNT 3
#define HMI_BUTTON_COUNT 4
#define TOTAL_BUTTON_COUNT 7

// Pin definitions
#define HMI_BUTTON_1_PIN 13
#define HMI_BUTTON_2_PIN 19
#define HMI_BUTTON_3_PIN 16
#define HMI_BUTTON_4_PIN 26

#define HMI_LED_POWER_PIN         17
#define HMI_LED_MOTORS_PIN        18
#define HMI_LED_COMMS_PIN         27
#define HMI_LED_WIFI_PIN          24
#define HMI_LED_BATTERY_GREEN_PIN 22
#define HMI_LED_BATTERY_RED_PIN   23
#define HMI_LED_USER_1_PIN        25
#define HMI_LED_USER_2_GREEN_PIN  6
#define HMI_LED_USER_2_RED_PIN    12

#define HMI_DISPLAY_RESET_PIN     2

#define UNKNOWN_IP "UNKNOWN"

enum class Turtlebot4Model
{
  LITE,
  STANDARD
};

static std::map<Turtlebot4Model, std::string> Turtlebot4ModelName
{
  {Turtlebot4Model::LITE, "Lite"},
  {Turtlebot4Model::STANDARD, "Standard"}
};

typedef std::function<void (void)> turtlebot4_function_callback_t;

}  // namespace turtlebot4

#endif  // TURTLEBOT4_NODE__UTILS_HPP_
