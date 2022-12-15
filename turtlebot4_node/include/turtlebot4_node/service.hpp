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

#include <memory>
#include <future>
#include <string>

#include <rclcpp/rclcpp.hpp>

namespace turtlebot4
{

// Base templated service class
template<typename ServiceT>
class Turtlebot4BaseService
{
public:
  // Constructor and Destructor
  explicit Turtlebot4BaseService(
    std::shared_ptr<rclcpp::Node> & nh, std::string service,
    uint32_t timeout)
  : nh_(nh), service_(service), timeout_(timeout)
  {
    // Create service client
    client_ = nh_->create_client<ServiceT>(service);
  }
  virtual ~Turtlebot4BaseService() {}

  virtual void make_request(std::shared_ptr<typename ServiceT::Request> request) = 0;

protected:
  // Node handle
  std::shared_ptr<rclcpp::Node> nh_;
  // Service
  std::string service_;
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t timeout_;
  // Service client
  typename rclcpp::Client<ServiceT>::SharedPtr client_;

  // Service response callback
  virtual void response_callback(typename rclcpp::Client<ServiceT>::SharedFuture future) = 0;
};

// Templated service class for services with a success and message response
template<typename ServiceT>
class Turtlebot4Service : public Turtlebot4BaseService<ServiceT>
{
public:
  // Constructor and Destructor
  explicit Turtlebot4Service(
    std::shared_ptr<rclcpp::Node> & nh, std::string service,
    uint32_t timeout = 30)
  : Turtlebot4BaseService<ServiceT>(nh, service, timeout)
  {}

  virtual ~Turtlebot4Service() {}

  void make_request(std::shared_ptr<typename ServiceT::Request> request) override
  {
    // Cancel existing timers
    if (this->timer_ != nullptr) {
      this->timer_->cancel();
    }

    auto timeout = rclcpp::Clock().now().seconds() + this->timeout_;

    // Create timer to check for service without blocking
    this->timer_ = this->nh_->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this, request, timeout]() -> void
      {
        // Check for service
        if (this->client_->wait_for_service(std::chrono::milliseconds(10))) {
          RCLCPP_INFO(
            this->nh_->get_logger(), "%s service available, sending request",
            this->service_.c_str());
          this->timer_->cancel();

          // Send request
          auto future_result = this->client_->async_send_request(
            request,
            std::bind(&Turtlebot4Service::response_callback, this, std::placeholders::_1));
        } else if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            this->nh_->get_logger(), "Interrupted while waiting for the service %s.",
            this->service_.c_str());
          this->timer_->cancel();
        } else if (rclcpp::Clock().now().seconds() > timeout) {  // Timed out
          RCLCPP_ERROR(
            this->nh_->get_logger(), "Service %s unavailable.",
            this->service_.c_str());
          this->timer_->cancel();
        }
      });
  }

private:
  // Service response callback
  void response_callback(typename rclcpp::Client<ServiceT>::SharedFuture future) override
  {
    auto result = future.get();
    RCLCPP_INFO(
      this->nh_->get_logger(), "%s service got results: %s",
      this->service_.c_str(), result->success ? "Success" : "Failed");
    RCLCPP_INFO(this->nh_->get_logger(), result->message.c_str());
  }
};

// Templated service class for services with an empty response
template<typename ServiceT>
class Turtlebot4EmptyService : public Turtlebot4BaseService<ServiceT>
{
public:
  // Constructor and Destructor
  explicit Turtlebot4EmptyService(
    std::shared_ptr<rclcpp::Node> & nh, std::string service,
    uint32_t timeout = 30)
  : Turtlebot4BaseService<ServiceT>(nh, service, timeout)
  {}

  virtual ~Turtlebot4EmptyService() {}

  void make_request(std::shared_ptr<typename ServiceT::Request> request) override
  {
    // Cancel existing timers
    if (this->timer_ != nullptr) {
      this->timer_->cancel();
    }

    auto timeout = rclcpp::Clock().now().seconds() + this->timeout_;

    // Create timer to check for service without blocking
    this->timer_ = this->nh_->create_wall_timer(
      std::chrono::milliseconds(1000),
      [this, request, timeout]() -> void
      {
        // Check for service
        if (this->client_->wait_for_service(std::chrono::milliseconds(10))) {
          RCLCPP_INFO(
            this->nh_->get_logger(), "%s service available, sending request",
            this->service_.c_str());
          this->timer_->cancel();

          // Send request
          auto future_result = this->client_->async_send_request(
            request,
            std::bind(&Turtlebot4EmptyService::response_callback, this, std::placeholders::_1));
        } else if (!rclcpp::ok()) {
          RCLCPP_ERROR(
            this->nh_->get_logger(), "Interrupted while waiting for the service %s.",
            this->service_.c_str());
          this->timer_->cancel();
        } else if (rclcpp::Clock().now().seconds() > timeout) {  // Timed out
          RCLCPP_ERROR(
            this->nh_->get_logger(), "Service %s unavailable.",
            this->service_.c_str());
          this->timer_->cancel();
        }
      });
  }

private:
  // Service response callback
  void response_callback(typename rclcpp::Client<ServiceT>::SharedFuture future) override
  {
    (void)future;
    RCLCPP_INFO(
      this->nh_->get_logger(), "%s service completed.", this->service_.c_str());
  }
};

}  // namespace turtlebot4

#endif  // TURTLEBOT4_NODE__SERVICE_HPP_
