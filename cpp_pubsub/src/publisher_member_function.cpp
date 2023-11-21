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
 * @file publisher_member_function.cpp
 * @author (Edited by) Suryavardhan Reddy Chappidi (chappidi@umd.edu)
 * @brief Publisher to publish custom messages using ROS 2.
 * @details This program defines a MinimalPublisher class for publishing 
 * custom messages in ROS 2. It also contains a service for updating the message.
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "cpp_pubsub/srv/update_message.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object.
   * 
   * @details Initializes the publisher, declares and reads the 'frequency' parameter, 
   * and sets up a timer for the callback. Also, initializes a service for updating messages.
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  try{
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    // Declare and get parameter
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Frequency of message publication.";
    this->declare_parameter("frequency", 2, param_desc);
    auto frequency = this->get_parameter("frequency").as_int();

    RCLCPP_INFO_STREAM(this->get_logger(), "Parameter frequency set to: " << frequency);

    if (frequency > 50) {
      RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger("minimal_publisher"),
                              "Frequency greater than fifty");
    } 

    // Setup timer for publishing messages
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / frequency),
        std::bind(&MinimalPublisher::timer_callback, this));

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Publisher");
    // Initialize service for message update
    server_ = this->create_service<cpp_pubsub::srv::UpdateMessage>(
        "service_node", std::bind(&MinimalPublisher::updateMessage, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialize the Server");
  } catch (...) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Error encountered at time of initialization!!"); 
      RCLCPP_FATAL_STREAM(this->get_logger(), "Publisher may not work!!");
    }
  }

  /**
   * @brief Handle incoming service requests to update the message.
   * 
   * @param request The service request containing the input message.
   * @param response The service response containing the updated message.
   */
  void updateMessage(const std::shared_ptr<cpp_pubsub::srv::UpdateMessage::Request> request,
                     std::shared_ptr<cpp_pubsub::srv::UpdateMessage::Response> response) {
    response->output = request->input + " Edited by service";
    RCLCPP_INFO(this->get_logger(), "Received request: '%s'", request->input.c_str());
    RCLCPP_INFO(this->get_logger(), "Sending response: '%s'", response->output.c_str());
  }
  

 private:
  /**
   * @brief Callback function for the timer event.
   * 
   * @details Publishes a message at a frequency determined by the 'frequency' parameter.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = server_response_message;
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Inserted message data");
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cpp_pubsub::srv::UpdateMessage>::SharedPtr server_;
  std::string server_response_message = "I am ROS2 Service";
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
