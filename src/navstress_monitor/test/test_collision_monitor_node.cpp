// Copyright 2026 Akshith Macharla
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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "navstress_interfaces/msg/clearance.hpp"
#include "navstress_monitor/collision_monitor_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using Clearance = navstress_interfaces::msg::Clearance;
using namespace std::chrono_literals;

class NodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
      {"scan_topics", std::vector<std::string>{"test_scan"}},
      {"footprint", std::string("[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]")},
    });
    monitor_ = std::make_shared<navstress_monitor::CollisionMonitorNode>(opts);
    helper_ = std::make_shared<rclcpp::Node>("test_helper");
    tf_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
    // The laser sits 0.2 m ahead of base_link.
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";
    tf.transform.translation.x = 0.2;
    tf.transform.rotation.w = 1.0;
    tf_pub_->sendTransform(tf);
    scan_pub_ =
      helper_->create_publisher<sensor_msgs::msg::LaserScan>("test_scan", rclcpp::SensorDataQoS());
    clearance_sub_ = helper_->create_subscription<Clearance>(
      "/navstress/clearance", rclcpp::SensorDataQoS(),
      [this](Clearance::ConstSharedPtr m) { clearances_.push_back(*m); });
    event_sub_ = helper_->create_subscription<Clearance>(
      "/navstress/collision_events", rclcpp::QoS(10).reliable(),
      [this](Clearance::ConstSharedPtr m) { events_.push_back(*m); });
    exec_.add_node(monitor_);
    exec_.add_node(helper_);
    // Let discovery and the static transform settle.
    spinFor(500ms);
  }

  void TearDown() override
  {
    exec_.remove_node(monitor_);
    exec_.remove_node(helper_);
  }

  void spinFor(std::chrono::milliseconds d)
  {
    const auto end = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < end) {
      exec_.spin_some(10ms);
    }
  }

  // One return straight ahead of the laser at range r.
  void sendScan(float r)
  {
    sensor_msgs::msg::LaserScan s;
    s.header.frame_id = "laser";
    s.header.stamp = helper_->now();
    s.angle_min = -0.1f;
    s.angle_max = 0.1f;
    s.angle_increment = 0.1f;
    s.range_min = 0.05f;
    s.range_max = 10.0f;
    s.ranges = {std::numeric_limits<float>::infinity(), r, std::numeric_limits<float>::infinity()};
    const size_t before = clearances_.size();
    scan_pub_->publish(s);
    const auto end = std::chrono::steady_clock::now() + 3s;
    while (clearances_.size() == before && std::chrono::steady_clock::now() < end) {
      exec_.spin_some(10ms);
    }
    ASSERT_GT(clearances_.size(), before) << "no clearance published";
  }

  rclcpp::executors::SingleThreadedExecutor exec_;
  std::shared_ptr<navstress_monitor::CollisionMonitorNode> monitor_;
  rclcpp::Node::SharedPtr helper_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<Clearance>::SharedPtr clearance_sub_;
  rclcpp::Subscription<Clearance>::SharedPtr event_sub_;
  std::vector<Clearance> clearances_;
  std::vector<Clearance> events_;
};

TEST_F(NodeTest, ClearanceUsesTransformAndFootprint)
{
  // Return at 0.6 m from the laser = 0.8 m from base_link = 0.3 m past the 0.5 m edge.
  sendScan(0.6f);
  const auto & c = clearances_.back();
  EXPECT_NEAR(c.min_clearance_m, 0.3, 1e-4);
  EXPECT_NEAR(c.closest_point.x, 0.8, 1e-4);
  EXPECT_EQ(c.header.frame_id, "base_link");
  EXPECT_FALSE(c.in_collision);
  EXPECT_TRUE(events_.empty());
}

TEST_F(NodeTest, OneEventPerCollisionNotPerScan)
{
  sendScan(0.31f);  // 0.01 m clearance: below the 0.02 m threshold
  sendScan(0.31f);
  sendScan(0.32f);
  spinFor(200ms);
  EXPECT_TRUE(clearances_.back().in_collision);
  ASSERT_EQ(events_.size(), 1u);
  EXPECT_NEAR(events_[0].min_clearance_m, 0.01, 1e-4);

  sendScan(1.0f);  // clear of the hysteresis band
  EXPECT_FALSE(clearances_.back().in_collision);
  sendScan(0.30f);  // touching again: a second event
  spinFor(200ms);
  EXPECT_EQ(events_.size(), 2u);
}
