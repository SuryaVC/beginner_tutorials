/**
 * @file service_client.cpp
 * @author Suryavardhan Reddy Chappidi (chappidi@umd.edu)
 * @brief Implementation of ServerClient class which creates a ROS node to send 
 * a new string to the server.
 * @details This class manages the communication with a ROS service to change a
 * string. It sends a new string to the server, waits for the response, and 
 * then outputs the changed string.
 * @version 0.1
 * @date 2023-11-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "cpp_pubsub/srv/UpdateMessage.srv"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/**
 * @class ServerClient
 * @brief ServerClient class that creates a node to send a new string to the server.
 * @details This class encapsulates the client-side functionality for a ROS 
 * service that changes a string. It creates and sends a request to the server 
 * and waits for a response.
 */
class ServerClient : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Server Client object
   * 
   * @details Initializes the client and connects to the 'service_node' service.
   */
  ServerClient() : Node("server_client") {
    client = this->create_client<beginner_tutorials::srv::UpdateMessage>(
        "service_node");
  }

  /**
   * @brief Create a request with the provided argument.
   * 
   * @param argv The command line arguments provided to the program.
   * @return A shared pointer to the created request.
   */
  auto getRequest(char **argv) {
    auto request = std::make_shared<
        beginner_tutorials::srv::UpdateMessage::Request>();
    request->input = argv[1];
    return request;
  }

  rclcpp::Client<beginner_tutorials::srv::UpdateMessage>::SharedPtr client;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<ServerClient> SClient = std::make_shared<ServerClient>();
  while (!SClient->client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }
  auto request = SClient->getRequest(argv);
  auto result = SClient->client->async_send_request(request);
  
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(SClient, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "change string '%s'",
                result.get()->output.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service change_string");
  }

  rclcpp::shutdown();
  return 0;
}
