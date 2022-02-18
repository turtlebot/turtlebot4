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

#ifndef TURTLEBOT4_NODE__SERVICE_HPP_
#define TURTLEBOT4_NODE__SERVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <future>
#include <string>

namespace turtlebot4
{
// Templated Service class to allow any ROS services to be used
template<typename ServiceT>
class Turtlebot4Service
{
public:
  // Constructor and Destructor
  explicit Turtlebot4Service(std::shared_ptr<rclcpp::Node> & nh, std::string service)
  : nh_(nh), service_(service)
  {
    // Create service client
    client_ = nh_->create_client<ServiceT>(service);
  }
  virtual ~Turtlebot4Service() {}

  void make_request(std::shared_ptr<typename ServiceT::Request> request)
  {
    // Cancel existing timers
    if (timer_ != nullptr) {
      timer_->cancel();
    }

    // Create timer to check for service without blocking
    timer_ = nh_->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this, request]() -> void
      {
        // Check for service
        if (client_->wait_for_service(std::chrono::milliseconds(10))) {
          RCLCPP_INFO(nh_->get_logger(), "%s service available, sending request", service_.c_str());
          timer_->cancel();

          // Send request
          auto future_result = client_->async_send_request(
            request,
            std::bind(&Turtlebot4Service::response_callback, this, std::placeholders::_1));
        } else if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            nh_->get_logger(), "Interrupted while waiting for the service %s.",
            service_.c_str());
          timer_->cancel();
        } else {
          // TODO(roni-kreinin): Add timeout
        }
      });
  }

private:
  // Node handle
  std::shared_ptr<rclcpp::Node> nh_;
  // Service
  std::string service_;
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  // Service client
  typename rclcpp::Client<ServiceT>::SharedPtr client_;

  // TODO(roni-kreinin): Virtual callback?
  // Service response callback
  void response_callback(typename rclcpp::Client<ServiceT>::SharedFuture future)
  {
    auto result = future.get();
    RCLCPP_INFO(
      nh_->get_logger(), "%s service got results: %s",
      service_.c_str(), result->success ? "Success" : "Failed");
    RCLCPP_INFO(nh_->get_logger(), result->message.c_str());
  }
};

}  // namespace turtlebot4

#endif  // TURTLEBOT4_NODE__SERVICE_HPP_
