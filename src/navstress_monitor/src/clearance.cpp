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

#include "navstress_monitor/clearance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

namespace navstress_monitor
{

bool pointInPolygon(const Point2 & p, const Polygon & poly)
{
  // Even-odd ray casting; works for convex and concave footprints alike.
  bool inside = false;
  const size_t n = poly.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const Point2 & a = poly[i];
    const Point2 & b = poly[j];
    if ((a.y > p.y) != (b.y > p.y)) {
      const double x_cross = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (p.x < x_cross) {
        inside = !inside;
      }
    }
  }
  return inside;
}

double distanceToSegment(const Point2 & p, const Point2 & a, const Point2 & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 0.0) {
    t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len2, 0.0, 1.0);
  }
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

double signedDistance(const Point2 & p, const Polygon & poly)
{
  double d = std::numeric_limits<double>::infinity();
  const size_t n = poly.size();
  for (size_t i = 0; i < n; ++i) {
    d = std::min(d, distanceToSegment(p, poly[i], poly[(i + 1) % n]));
  }
  return pointInPolygon(p, poly) ? -d : d;
}

ClearanceResult minClearance(
  const std::vector<Point2> & points, const Polygon & footprint, bool ignore_inside)
{
  ClearanceResult out;
  out.min_clearance = std::numeric_limits<double>::infinity();
  for (const auto & p : points) {
    const double d = signedDistance(p, footprint);
    if (ignore_inside && d < 0.0) {
      continue;
    }
    if (d < out.min_clearance) {
      out.min_clearance = d;
      out.closest = p;
      out.valid = true;
    }
  }
  return out;
}

CollisionTracker::CollisionTracker(double threshold_m, double hysteresis_m)
: threshold_m_(threshold_m), hysteresis_m_(hysteresis_m)
{
  if (hysteresis_m < 0.0) {
    throw std::invalid_argument("hysteresis must be >= 0");
  }
}

bool CollisionTracker::update(double clearance_m)
{
  if (!in_collision_ && clearance_m < threshold_m_) {
    in_collision_ = true;
    ++count_;
    return true;
  }
  if (in_collision_ && clearance_m > threshold_m_ + hysteresis_m_) {
    in_collision_ = false;
  }
  return false;
}

void CollisionTracker::reset()
{
  in_collision_ = false;
  count_ = 0;
}

Polygon parseFootprint(const std::string & text)
{
  static const std::regex pair_re(R"(\[\s*([-+0-9.eE]+)\s*,\s*([-+0-9.eE]+)\s*\])");
  Polygon poly;
  for (auto it = std::sregex_iterator(text.begin(), text.end(), pair_re);
       it != std::sregex_iterator(); ++it) {
    try {
      poly.push_back({std::stod((*it)[1].str()), std::stod((*it)[2].str())});
    } catch (const std::exception &) {
      throw std::invalid_argument("bad number in footprint: " + it->str());
    }
  }
  if (poly.size() < 3) {
    throw std::invalid_argument("footprint needs at least 3 [x, y] points: " + text);
  }
  return poly;
}

}  // namespace navstress_monitor
