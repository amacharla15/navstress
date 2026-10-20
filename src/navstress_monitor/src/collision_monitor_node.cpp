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

#include "navstress_monitor/collision_monitor_node.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace navstress_monitor
{

CollisionMonitorNode::CollisionMonitorNode(const rclcpp::NodeOptions & options)
: Node("navstress_collision_monitor", options),
  tracker_(
    declare_parameter("collision_threshold_m", 0.02), declare_parameter("hysteresis_m", 0.05))
{
  base_frame_ = declare_parameter("base_frame", std::string("base_link"));
  // Same polygon Nav2 uses for Carter (carter_navigation_params.yaml) by default.
  footprint_ = parseFootprint(declare_parameter(
    "footprint", std::string("[[0.14, 0.25], [0.14, -0.25], [-0.607, -0.25], [-0.607, 0.25]]")));
  ignore_inside_ = declare_parameter("ignore_points_inside_footprint", true);
  source_timeout_s_ = declare_parameter("source_timeout_s", 0.5);
  max_range_m_ = declare_parameter("max_range_m", 10.0);
  cloud_min_z_ = declare_parameter("cloud_min_z", 0.05);
  cloud_max_z_ = declare_parameter("cloud_max_z", 2.0);
  const auto scan_topics = declare_parameter("scan_topics", std::vector<std::string>{"scan"});
  const auto cloud_topics = declare_parameter("cloud_topics", std::vector<std::string>{""});

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Isaac Sim publishes sensors and odometry with Sensor Data QoS (best effort); a
  // reliable subscription would never match.
  const auto sensor_qos = rclcpp::SensorDataQoS();
  for (const auto & topic : scan_topics) {
    if (topic.empty()) {
      continue;
    }
    const size_t index = sources_.size();
    sources_.emplace_back();
    subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(
      topic, sensor_qos, [this, index](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
        onScan(msg, index);
      }));
  }
  for (const auto & topic : cloud_topics) {
    if (topic.empty()) {
      continue;
    }
    const size_t index = sources_.size();
    sources_.emplace_back();
    subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, sensor_qos, [this, index](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        onCloud(msg, index);
      }));
  }
  if (sources_.empty()) {
    throw std::invalid_argument("no scan_topics or cloud_topics configured");
  }
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    declare_parameter("odom_topic", std::string("odom")), sensor_qos,
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { onOdom(msg); });

  // Clearance is a stream where only the newest value matters: best effort. Collision
  // events are rare and must arrive: reliable, keep 10.
  clearance_pub_ = create_publisher<navstress_interfaces::msg::Clearance>(
    "/navstress/clearance", rclcpp::SensorDataQoS());
  event_pub_ = create_publisher<navstress_interfaces::msg::Clearance>(
    "/navstress/collision_events", rclcpp::QoS(10).reliable());
}

bool CollisionMonitorNode::lookup(const std::string & frame, tf2::Transform & out)
{
  try {
    // Latest rather than the message stamp: sensors are rigidly mounted, so the
    // sensor->base transform is static and any stamp gives the same answer.
    const auto msg = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
    tf2::fromMsg(msg.transform, out);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "no transform %s -> %s: %s", frame.c_str(),
      base_frame_.c_str(), ex.what());
    return false;
  }
}

void CollisionMonitorNode::onScan(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg, size_t index)
{
  tf2::Transform tf;
  if (!lookup(msg->header.frame_id, tf)) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  std::vector<Point2> pts;
  pts.reserve(msg->ranges.size());
  const double max_r = std::min<double>(msg->range_max, max_range_m_);
  for (size_t i = 0; i < msg->ranges.size(); ++i) {
    const double r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > max_r) {
      continue;
    }
    const double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
    const tf2::Vector3 p = tf * tf2::Vector3(r * std::cos(a), r * std::sin(a), 0.0);
    pts.push_back({p.x(), p.y()});
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sources_[index] = Source{std::move(pts), stamp, true};
  }
  publish(stamp);
}

void CollisionMonitorNode::onCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg, size_t index)
{
  tf2::Transform tf;
  if (!lookup(msg->header.frame_id, tf)) {
    return;
  }
  const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
  std::vector<Point2> pts;
  pts.reserve(static_cast<size_t>(msg->width) * msg->height);
  sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
  for (; x != x.end(); ++x, ++y, ++z) {
    if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
      continue;
    }
    const tf2::Vector3 sensor_p(*x, *y, *z);
    if (sensor_p.length() > max_range_m_) {
      continue;
    }
    const tf2::Vector3 p = tf * sensor_p;
    // Floor returns and overhead structure aren't obstacles to the base.
    if (p.z() < cloud_min_z_ || p.z() > cloud_max_z_) {
      continue;
    }
    pts.push_back({p.x(), p.y()});
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sources_[index] = Source{std::move(pts), stamp, true};
  }
  publish(stamp);
}

void CollisionMonitorNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  speed_mps_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
}

void CollisionMonitorNode::publish(const rclcpp::Time & stamp)
{
  navstress_interfaces::msg::Clearance out;
  bool new_collision = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Point2> all;
    for (const auto & s : sources_) {
      // A source that stopped publishing must not keep reporting an old obstacle.
      if (!s.has_data || (stamp - s.stamp).seconds() > source_timeout_s_) {
        continue;
      }
      all.insert(all.end(), s.points.begin(), s.points.end());
    }
    const ClearanceResult r = minClearance(all, footprint_, ignore_inside_);
    out.header.stamp = stamp;
    out.header.frame_id = base_frame_;
    out.robot_speed_mps = speed_mps_;
    if (r.valid) {
      out.min_clearance_m = r.min_clearance;
      out.closest_point.x = r.closest.x;
      out.closest_point.y = r.closest.y;
      new_collision = tracker_.update(r.min_clearance);
    } else {
      out.min_clearance_m = std::numeric_limits<double>::infinity();
    }
    out.in_collision = tracker_.inCollision();
  }
  clearance_pub_->publish(out);
  if (new_collision) {
    RCLCPP_WARN(
      get_logger(), "collision: clearance %.3f m at (%.2f, %.2f), speed %.2f m/s",
      out.min_clearance_m, out.closest_point.x, out.closest_point.y, out.robot_speed_mps);
    event_pub_->publish(out);
  }
}

}  // namespace navstress_monitor
