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

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "navstress_monitor/clearance.hpp"

using navstress_monitor::CollisionTracker;
using navstress_monitor::distanceToSegment;
using navstress_monitor::minClearance;
using navstress_monitor::parseFootprint;
using navstress_monitor::Point2;
using navstress_monitor::pointInPolygon;
using navstress_monitor::Polygon;
using navstress_monitor::signedDistance;

namespace
{
// 1 m x 1 m square centred on the origin.
const Polygon kSquare{{0.5, 0.5}, {0.5, -0.5}, {-0.5, -0.5}, {-0.5, 0.5}};
// Nova Carter's Nav2 footprint: base_link sits 0.14 m behind the front edge.
const Polygon kCarter{{0.14, 0.25}, {0.14, -0.25}, {-0.607, -0.25}, {-0.607, 0.25}};
}  // namespace

TEST(PointInPolygon, InsideOutsideAndWindingAgnostic)
{
  EXPECT_TRUE(pointInPolygon({0.0, 0.0}, kSquare));
  EXPECT_TRUE(pointInPolygon({0.49, -0.49}, kSquare));
  EXPECT_FALSE(pointInPolygon({0.51, 0.0}, kSquare));
  EXPECT_FALSE(pointInPolygon({0.0, -2.0}, kSquare));
  const Polygon reversed(kSquare.rbegin(), kSquare.rend());
  EXPECT_TRUE(pointInPolygon({0.1, 0.2}, reversed));
}

TEST(PointInPolygon, ConcaveFootprint)
{
  // L-shape: the notch at (0.75, 0.75) is outside.
  const Polygon l_shape{{0, 0}, {1, 0}, {1, 0.5}, {0.5, 0.5}, {0.5, 1}, {0, 1}};
  EXPECT_TRUE(pointInPolygon({0.25, 0.75}, l_shape));
  EXPECT_FALSE(pointInPolygon({0.75, 0.75}, l_shape));
}

TEST(DistanceToSegment, ProjectsAndClampsToEndpoints)
{
  EXPECT_DOUBLE_EQ(distanceToSegment({0.5, 1.0}, {0, 0}, {1, 0}), 1.0);
  EXPECT_DOUBLE_EQ(distanceToSegment({2.0, 0.0}, {0, 0}, {1, 0}), 1.0);
  EXPECT_DOUBLE_EQ(distanceToSegment({-3.0, 4.0}, {0, 0}, {1, 0}), 5.0);
  EXPECT_DOUBLE_EQ(distanceToSegment({1.0, 1.0}, {0, 0}, {0, 0}), std::sqrt(2.0));
}

TEST(SignedDistance, NegativeInsidePositiveOutside)
{
  EXPECT_NEAR(signedDistance({1.0, 0.0}, kSquare), 0.5, 1e-12);
  EXPECT_NEAR(signedDistance({0.3, 0.0}, kSquare), -0.2, 1e-12);
  // Diagonal from a corner: distance to the corner itself.
  EXPECT_NEAR(signedDistance({1.5, 1.5}, kSquare), std::sqrt(2.0), 1e-12);
  // A point 0.30 m ahead of Carter's front edge.
  EXPECT_NEAR(signedDistance({0.44, 0.0}, kCarter), 0.30, 1e-12);
}

TEST(MinClearance, PicksClosestAndOptionallyIgnoresSelfReturns)
{
  const std::vector<Point2> pts{{2.0, 0.0}, {0.0, 0.9}, {0.1, 0.1}};
  const auto with_inside = minClearance(pts, kSquare, false);
  ASSERT_TRUE(with_inside.valid);
  EXPECT_NEAR(with_inside.min_clearance, -0.4, 1e-12);

  const auto ignored = minClearance(pts, kSquare, true);
  ASSERT_TRUE(ignored.valid);
  EXPECT_NEAR(ignored.min_clearance, 0.4, 1e-12);
  EXPECT_DOUBLE_EQ(ignored.closest.y, 0.9);
}

TEST(MinClearance, NoPointsIsInvalidNotZero)
{
  const auto r = minClearance({}, kSquare, true);
  EXPECT_FALSE(r.valid);
  EXPECT_TRUE(std::isinf(r.min_clearance));
}

TEST(CollisionTracker, CountsRisingEdgesWithHysteresis)
{
  CollisionTracker t(0.02, 0.05);
  EXPECT_FALSE(t.update(0.30));
  EXPECT_TRUE(t.update(0.01));    // enters collision
  EXPECT_FALSE(t.update(0.015));  // still in it
  EXPECT_FALSE(t.update(0.05));   // above threshold but inside the hysteresis band
  EXPECT_TRUE(t.inCollision());
  EXPECT_FALSE(t.update(0.08));  // leaves
  EXPECT_FALSE(t.inCollision());
  EXPECT_TRUE(t.update(0.0));  // second, separate collision
  EXPECT_EQ(t.count(), 2u);
  t.reset();
  EXPECT_EQ(t.count(), 0u);
  EXPECT_THROW(CollisionTracker(0.02, -1.0), std::invalid_argument);
}

TEST(ParseFootprint, Nav2StringFormat)
{
  const auto p = parseFootprint("[ [0.14, 0.25], [0.14, -0.25], [-0.607, -0.25], [-0.607, 0.25] ]");
  ASSERT_EQ(p.size(), 4u);
  EXPECT_DOUBLE_EQ(p[2].x, -0.607);
  EXPECT_DOUBLE_EQ(p[1].y, -0.25);
  EXPECT_THROW(parseFootprint("[[0, 0], [1, 1]]"), std::invalid_argument);
  EXPECT_THROW(parseFootprint("not a footprint"), std::invalid_argument);
}
