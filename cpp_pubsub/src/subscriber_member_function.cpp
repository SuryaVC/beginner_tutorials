// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file subscriber_member_function.cpp
 * @author (Edited by) Suryavardhan Reddy Chappidi (chappidi@umd.edu)
 * @brief Subscriber to subscribe to messages using ROS 2.
 * @details This program defines a MinimalSubscriber class for subscribing to
 * custom messages in ROS 2. It listens to the messages published on a specific
 * topic and logs them.
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object.
   *
   * @details Initializes the subscriber to the 'topic'. The subscriber listens
   * for messages of type std_msgs::msg::String and processes them using a
   * callback function.
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    try {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
          "chatter", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      RCLCPP_INFO(this->get_logger(), "Subscriber initialized successfully");
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Initialization error in subscriber");
      RCLCPP_FATAL(this->get_logger(), "Subscriber may not function properly");
    }
  }

 private:
  /**
   * @brief Callback function for the subscriber.
   *
   * @param msg The message received from the publisher.
   * @details This function logs the received message.
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "Shutting down subscriber node");
  return 0;
}
