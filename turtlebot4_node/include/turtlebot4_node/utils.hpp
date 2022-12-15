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

static constexpr auto CREATE3_BUTTON_COUNT = 3;
static constexpr auto HMI_BUTTON_COUNT = 4;
static constexpr auto TOTAL_BUTTON_COUNT = 7;

static constexpr auto UNKNOWN_IP = "UNKNOWN";

enum Turtlebot4LedEnum
{
  POWER,
  MOTORS,
  COMMS,
  WIFI,
  BATTERY,
  USER_1,
  USER_2,
  COUNT
};

enum Turtlebot4LedType
{
  GREEN_ONLY,
  RED_GREEN
};

enum Turtlebot4LedColor
{
  OFF = 0,
  GREEN = 1,
  RED = 2,
  YELLOW = 3
};

enum class Turtlebot4Model
{
  LITE,
  STANDARD
};

static std::map<Turtlebot4Model, std::string> Turtlebot4ModelName
{
  {Turtlebot4Model::LITE, "lite"},
  {Turtlebot4Model::STANDARD, "standard"}
};

typedef std::function<void (void)> turtlebot4_function_callback_t;
typedef std::function<void (std::string)> turtlebot4_function_call_callback_t;

}  // namespace turtlebot4

#endif  // TURTLEBOT4_NODE__UTILS_HPP_
