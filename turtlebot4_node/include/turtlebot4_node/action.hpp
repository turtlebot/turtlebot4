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

#ifndef TURTLEBOT4_NODE__ACTION_HPP_
#define TURTLEBOT4_NODE__ACTION_HPP_

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"


namespace turtlebot4
{
// Templated Action class to allow any ROS actions to be used
template<typename ActionT>
class Turtlebot4Action
{
public:
  // Constructor and Destructor
  explicit Turtlebot4Action(std::shared_ptr<rclcpp::Node> & nh, std::string action)
  : nh_(nh), action_(action)
  {
    // Create action client
    client_ = rclcpp_action::create_client<ActionT>(nh_, action);
  }
  virtual ~Turtlebot4Action() {}

  void send_goal()
  {
    RCLCPP_INFO(nh_->get_logger(), "Waiting for %s action server", action_.c_str());

    // Cancel existing timers
    if (timer_ != nullptr) {
      timer_->cancel();
    }

    // Create timer to check for service without blocking
    timer_ = nh_->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this]() -> void
      {
        // Wait for action server
        if (client_->wait_for_action_server(std::chrono::milliseconds(10))) {
          RCLCPP_INFO(
            nh_->get_logger(), "%s action server available, sending goal",
            action_.c_str());
          timer_->cancel();

          // Create goal
          auto goal_msg = typename ActionT::Goal();

          // Set goal callbacks
          auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
          send_goal_options.goal_response_callback =
          std::bind(&Turtlebot4Action::goal_response_callback, this, std::placeholders::_1);
          send_goal_options.feedback_callback =
          std::bind(
            &Turtlebot4Action::feedback_callback, this, std::placeholders::_1,
            std::placeholders::_2);
          send_goal_options.result_callback =
          std::bind(&Turtlebot4Action::result_callback, this, std::placeholders::_1);

          // Send goal
          this->client_->async_send_goal(goal_msg, send_goal_options);
        } else {
          // TODO(roni-kreinin): Add timeout
        }
      });
  }

  void send_goal(std::shared_ptr<typename ActionT::Goal> goal_msg)
  {
    RCLCPP_INFO(nh_->get_logger(), "Waiting for %s action server", action_.c_str());

    // Cancel existing timers
    if (timer_ != nullptr) {
      timer_->cancel();
    }

    // Create timer to check for service without blocking
    timer_ = nh_->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this, goal_msg]() -> void
      {
        // Wait for action server
        if (client_->wait_for_action_server(std::chrono::milliseconds(10))) {
          RCLCPP_INFO(
            nh_->get_logger(), "%s action server available, sending goal",
            action_.c_str());
          timer_->cancel();

          // Set goal callbacks
          auto send_goal_options = typename rclcpp_action::Client<ActionT>::SendGoalOptions();
          send_goal_options.goal_response_callback =
          std::bind(&Turtlebot4Action::goal_response_callback, this, std::placeholders::_1);
          send_goal_options.feedback_callback =
          std::bind(
            &Turtlebot4Action::feedback_callback, this, std::placeholders::_1,
            std::placeholders::_2);
          send_goal_options.result_callback =
          std::bind(&Turtlebot4Action::result_callback, this, std::placeholders::_1);

          // Send goal
          this->client_->async_send_goal(*goal_msg, send_goal_options);
        } else {
          // TODO(roni-kreinin): Add timeout
        }
      });
  }

private:
  // Node handle
  std::shared_ptr<rclcpp::Node> nh_;
  // Action
  std::string action_;
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  // Action client
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;

  // TODO(roni-kreinin): Virtual callbacks?

  /**
    * @brief Goal response callback
    * @input future - Shared future with goal response
    */
  void goal_response_callback(
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(nh_->get_logger(), "%s goal was rejected by server", action_.c_str());
    } else {
      RCLCPP_INFO(
        nh_->get_logger(), "%s goal accepted by server, waiting for result",
        action_.c_str());
    }
  }

  /**
    * @brief Feedback callback
    * @input ptr - Unused
    * @input feedback - Action feedback
    */
  void feedback_callback(
    typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr ptr,
    const std::shared_ptr<const typename ActionT::Feedback> feedback)
  {
    (void)ptr;
    (void)feedback;
  }

  /**
    * @brief Result callback
    * @input result - Action result
    */
  void result_callback(
    const typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(nh_->get_logger(), "%s goal succeeded", action_.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(nh_->get_logger(), "%s goal was aborted", action_.c_str());
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(nh_->get_logger(), "%s goal was canceled", action_.c_str());
        return;
      default:
        RCLCPP_ERROR(nh_->get_logger(), "%s Unknown result code", action_.c_str());
        return;
    }
  }
};

}  // namespace turtlebot4

#endif  // TURTLEBOT4_NODE__ACTION_HPP_
