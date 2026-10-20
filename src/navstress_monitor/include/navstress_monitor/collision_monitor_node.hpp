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

#ifndef NAVSTRESS_MONITOR__COLLISION_MONITOR_NODE_HPP_
#define NAVSTRESS_MONITOR__COLLISION_MONITOR_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "navstress_interfaces/msg/clearance.hpp"
#include "navstress_monitor/clearance.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace navstress_monitor
{

// Measures clearance between range data and the robot footprint and reports
// collisions. Named navstress_collision_monitor because Carter's Nav2 config already
// runs Nav2's own node called collision_monitor.
class CollisionMonitorNode : public rclcpp::Node
{
public:
  explicit CollisionMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct Source
  {
    std::vector<Point2> points;
    rclcpp::Time stamp;
    bool has_data{false};
  };

  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr & msg, size_t index);
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg, size_t index);
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  bool lookup(const std::string & frame, tf2::Transform & out);
  void publish(const rclcpp::Time & stamp);

  std::string base_frame_;
  Polygon footprint_;
  bool ignore_inside_;
  double source_timeout_s_;
  double max_range_m_;
  double cloud_min_z_;
  double cloud_max_z_;

  std::mutex mutex_;
  std::vector<Source> sources_;
  CollisionTracker tracker_;
  double speed_mps_{0.0};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<navstress_interfaces::msg::Clearance>::SharedPtr clearance_pub_;
  rclcpp::Publisher<navstress_interfaces::msg::Clearance>::SharedPtr event_pub_;
};

}  // namespace navstress_monitor

#endif  // NAVSTRESS_MONITOR__COLLISION_MONITOR_NODE_HPP_
