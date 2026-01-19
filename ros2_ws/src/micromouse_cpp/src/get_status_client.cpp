/**
 * @file get_status_client.cpp
 * @brief Implementation of Service Client for Robot Status
 *
 * Complete the TODOs to implement the service client
 *
 * This file demonstrates how to:
 * - Create a ROS2 service client
 * - Wait for a service to become available
 * - Send asynchronous service requests
 * - Handle service responses with callbacks
 *
 * Point values are shown in each TODO comment.
 */

#include "micromouse_cpp/get_status_client.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace micromouse {

// =============================================================================
// Constructor
// =============================================================================

GetStatusClient::GetStatusClient() : Node("get_status_client") {
  client_ = this->create_client<GetRobotStatus>("/get_robot_status");

  RCLCPP_INFO(this->get_logger(),
              "Service client created for /get_robot_status");
}

// =============================================================================
// send_request
// =============================================================================

void GetStatusClient::send_request() {
  // Wait for service to be available
  RCLCPP_INFO(this->get_logger(), "Waiting for service...");

  if (!client_->wait_for_service(5s)) {
    RCLCPP_WARN(this->get_logger(),
                "Service /get_robot_status not available after 5 seconds");
    RCLCPP_WARN(this->get_logger(),
                "Make sure micromouse_node is running in MMS simulator");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Service available, sending request...");
  auto request = std::make_shared<GetRobotStatus::Request>();

  // Send the request asynchronously with callback
  client_->async_send_request(
      request,
      std::bind(&GetStatusClient::response_callback, this, _1));
}

// =============================================================================
// response_callback
// =============================================================================

void GetStatusClient::response_callback(
    rclcpp::Client<GetRobotStatus>::SharedFuture future) {
   if (future.wait_for(0s) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed (future not ready)");
    rclcpp::shutdown();
    return;
  }

  const auto& response = future.get();

  RCLCPP_INFO(this->get_logger(), "--- Robot Status Response ---");
  RCLCPP_INFO(this->get_logger(), "  Position: (%d, %d)",
              response->position_x, response->position_y);
  RCLCPP_INFO(this->get_logger(), "  Direction: %s", response->direction.c_str());
  RCLCPP_INFO(this->get_logger(), "  Steps Taken: %d", response->steps_taken);
  RCLCPP_INFO(this->get_logger(), "  Steps to Goal Est: %d",
              response->steps_to_goal_estimate);
  RCLCPP_INFO(this->get_logger(), "  Elapsed Time: %.2fs",
              response->elapsed_seconds);
  RCLCPP_INFO(this->get_logger(), "  Is Running: %s",
              response->is_running ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Success: %s",
              response->success ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  Message: %s", response->message.c_str());

  rclcpp::shutdown();
}

}  // namespace micromouse

// =============================================================================
// Main
// =============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<micromouse::GetStatusClient>();

  // Send request after node is initialized
  node->send_request();

  // Spin to process callback
  rclcpp::spin(node);
}
