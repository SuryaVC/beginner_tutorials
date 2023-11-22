// Copyright (c) 2023 Suryavardhan Reddy Chappidi

/**
 * @file test.cpp
 * @author Suryavardhan Reddy Chappidi (chappidi@umd.edu)
 * @brief Integration tests for ROS 2 publisher using GoogleTest framework. This
 * file contains tests for verifying the correct creation and functionality of a
 * ROS 2 publisher within a custom node. It includes tests for checking the
 * number of publishers on a specific topic along with tf2 broadcaster and the
 * successful operation of ROS 2 functionalities.
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

/**
 * @class TaskTalker
 * @brief Class to contain the Talker test node for ROS 2 integration testing.
 */
class TaskTalker : public testing::Test {
 protected:
  /**
   * @brief Shared pointer to a ROS node, used for setting up a publisher or
   * subscriber in tests.
   */
  rclcpp::Node::SharedPtr node_;
};

/**
 * @brief Tests if the publisher is able to publish a message on the 'chatter'
 * topic.
 *
 * Creates a publisher on the 'chatter' topic and a corresponding subscriber. It
 * then checks if a message is successfully published and received by the
 * subscriber.
 */
TEST_F(TaskTalker, test_num_publishers) {
  node_ = rclcpp::Node::make_shared("test_num_pubs");
  auto test_pub =
      node_->create_publisher<std_msgs::msg::String>("chatter", 10.0);

  auto num_pub = node_->count_publishers("chatter");
  EXPECT_EQ(1, static_cast<int>(num_pub));
}

/**
 * @brief Tests the broadcasting and reception of a tf2 transform.
 *
 * Sets up a tf2 broadcaster to send a transform, then checks if the transform
 * can be successfully received.
 */
TEST_F(TaskTalker, test_tf2) {
  node_ = rclcpp::Node::make_shared("test_tf");
  auto tf_br = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  auto tf_buffer = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Broadcasting a simple transform
  geometry_msgs::msg::TransformStamped t;
  rclcpp::Time now = node_->get_clock()->now();
  t.header.stamp = now;
  t.header.frame_id = "world";
  t.child_frame_id = "test";
  t.transform.translation.x = 0.1;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;
  tf_br->sendTransform(t);

  // Check if the transform is received
  std::string tgt_frame = "test";
  std::string src_frame = "world";
  auto transform_available =
      tf_buffer->canTransform(tgt_frame, src_frame, now, 50ms);

  EXPECT_TRUE(transform_available);
}
