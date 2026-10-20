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

#ifndef NAVSTRESS_MONITOR__CLEARANCE_HPP_
#define NAVSTRESS_MONITOR__CLEARANCE_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace navstress_monitor
{

struct Point2
{
  double x;
  double y;
};

// Robot footprint in the base frame, vertices in order (either winding).
using Polygon = std::vector<Point2>;

bool pointInPolygon(const Point2 & p, const Polygon & poly);

double distanceToSegment(const Point2 & p, const Point2 & a, const Point2 & b);

// Distance from p to the polygon boundary; negative when p is inside.
double signedDistance(const Point2 & p, const Polygon & poly);

struct ClearanceResult
{
  bool valid{false};  // false when no point was considered
  double min_clearance{0.0};
  Point2 closest{0.0, 0.0};
};

// Smallest signed distance over the points. Points inside the footprint are skipped
// when ignore_inside is set: on a real robot they are returns from the robot's own
// body, which would otherwise read as a permanent collision.
ClearanceResult minClearance(
  const std::vector<Point2> & points, const Polygon & footprint, bool ignore_inside);

// Counts collisions as rising edges with hysteresis, so a clearance hovering at the
// threshold is one collision, not one per scan.
class CollisionTracker
{
public:
  CollisionTracker(double threshold_m, double hysteresis_m);
  // Returns true exactly when a new collision starts.
  bool update(double clearance_m);
  bool inCollision() const { return in_collision_; }
  uint32_t count() const { return count_; }
  void reset();

private:
  double threshold_m_;
  double hysteresis_m_;
  bool in_collision_{false};
  uint32_t count_{0};
};

// Parses a Nav2-style footprint string such as "[[0.14, 0.25], [0.14, -0.25], ...]".
// Throws std::invalid_argument for anything that isn't at least three [x, y] pairs.
Polygon parseFootprint(const std::string & text);

}  // namespace navstress_monitor

#endif  // NAVSTRESS_MONITOR__CLEARANCE_HPP_
