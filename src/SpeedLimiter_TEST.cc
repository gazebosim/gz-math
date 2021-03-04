/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include "ignition/math/Helpers.hh"
#include "ignition/math/SpeedLimiter.hh"

using namespace ignition;
using namespace math;
using namespace std::literals::chrono_literals;

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, Default)
{
  SpeedLimiter limiter;

  double vel{5.0};
  EXPECT_DOUBLE_EQ(1.0, limiter.Limit(vel, 4.0, 3.0, 1ms));
  EXPECT_DOUBLE_EQ(5.0, vel);

  EXPECT_DOUBLE_EQ(1.0, limiter.LimitVelocity(vel));
  EXPECT_DOUBLE_EQ(5.0, vel);

  EXPECT_DOUBLE_EQ(1.0, limiter.LimitAcceleration(vel, 4.0, 1ms));
  EXPECT_DOUBLE_EQ(5.0, vel);

  EXPECT_DOUBLE_EQ(1.0, limiter.LimitJerk(vel, 4.0, 3.0, 1ms));
  EXPECT_DOUBLE_EQ(5.0, vel);

  vel = 0.0;
  EXPECT_DOUBLE_EQ(1.0, limiter.Limit(vel, 4.0, 3.0, 1ms));
  EXPECT_DOUBLE_EQ(0.0, vel);
}

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, LimitVelocity)
{
  double minVel = -1.0;
  double maxVel = 4.0;
  SpeedLimiter limiter(true, false, false, minVel, maxVel);

  // Within bounds
  double vel{1.0};
  EXPECT_DOUBLE_EQ(1.0, limiter.LimitVelocity(vel));
  EXPECT_DOUBLE_EQ(1.0, vel);

  // Above upper bound
  vel = 5.0;
  EXPECT_DOUBLE_EQ(0.8, limiter.LimitVelocity(vel));
  EXPECT_DOUBLE_EQ(maxVel, vel);

  // Under lower bound
  vel = -2.0;
  EXPECT_DOUBLE_EQ(0.5, limiter.LimitVelocity(vel));
  EXPECT_DOUBLE_EQ(minVel, vel);
}

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, LimitAcceleration)
{
  double minAcc = -2.0;
  double maxAcc = 4.0;
  SpeedLimiter limiter(false, true, false, -INF_D, INF_D, minAcc, maxAcc);

  auto dt = 1s;

  // Within bounds
  double vel{2.0};
  double prevVel = 1.0;
  EXPECT_DOUBLE_EQ(1.0, limiter.LimitAcceleration(vel, prevVel, dt));
  EXPECT_DOUBLE_EQ(2.0, vel);

  // Above upper bound
  vel = 10.0;
  prevVel = 1.0;
  EXPECT_DOUBLE_EQ(0.5, limiter.LimitAcceleration(vel, prevVel, dt));
  EXPECT_DOUBLE_EQ(5.0, vel);
  EXPECT_DOUBLE_EQ(prevVel + maxAcc, vel);
  EXPECT_DOUBLE_EQ(vel - prevVel, maxAcc);

  // Under lower bound
  vel = -8.0;
  prevVel = -2.0;
  EXPECT_DOUBLE_EQ(0.5, limiter.LimitAcceleration(vel, prevVel, dt));
  EXPECT_DOUBLE_EQ(-4.0, vel);
  EXPECT_DOUBLE_EQ(prevVel + minAcc, vel);
  EXPECT_DOUBLE_EQ(vel - prevVel, minAcc);
}

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, LimitJerk)
{
  double minJerk = -1.0;
  double maxJerk = 1.0;
  SpeedLimiter limiter(false, false, true, -INF_D, INF_D, -INF_D, INF_D,
      minJerk, maxJerk);

  auto dt = 1s;

  // Within bounds
  double vel{2.0};
  double prevVel = 1.0;
  double prevPrevVel = 0.5;
  EXPECT_DOUBLE_EQ(1.0, limiter.LimitJerk(vel, prevVel, prevPrevVel, dt));
  EXPECT_DOUBLE_EQ(2.0, vel);

  // Above upper bound (desired: accel 4.0, jerk 3.0)
  vel = 6.0;
  prevVel = 2.0;
  prevPrevVel = 1.0;
  EXPECT_DOUBLE_EQ(4.0/6.0, limiter.LimitJerk(vel, prevVel, prevPrevVel, dt));
  EXPECT_DOUBLE_EQ(4.0, vel);

  double accel = vel - prevVel;
  double prevAccel = prevVel - prevPrevVel;
  EXPECT_DOUBLE_EQ(accel - prevAccel, maxJerk);

  // Under lower bound
  vel = -6.0;
  prevVel = -2.0;
  prevPrevVel = -1.0;
  EXPECT_DOUBLE_EQ(4.0/6.0, limiter.LimitJerk(vel, prevVel, prevPrevVel, dt));
  EXPECT_DOUBLE_EQ(-4.0, vel);

  accel = vel - prevVel;
  prevAccel = prevVel - prevPrevVel;
  EXPECT_DOUBLE_EQ(accel - prevAccel, minJerk);
}

/////////////////////////////////////////////////
TEST(SpeedLimiterTest, LimitAll)
{
  double minVel = -4.0;
  double maxVel = 4.0;
  double minAcc = -2.0;
  double maxAcc = 2.0;
  double minJerk = -1.0;
  double maxJerk = 1.0;
  SpeedLimiter limiter(true, true, true, minVel, maxVel, minAcc, maxAcc,
      minJerk, maxJerk);

  auto dt = 1s;

  // Within bounds
  double vel{2.0};
  double prevVel = 1.0;
  double prevPrevVel = 0.5;
  EXPECT_DOUBLE_EQ(1.0, limiter.LimitJerk(vel, prevVel, prevPrevVel, dt));
  EXPECT_DOUBLE_EQ(2.0, vel);

  // Above upper velocity bound
  vel = 5.0;
  prevVel = 2.0;
  prevPrevVel = 1.0;
  EXPECT_DOUBLE_EQ(4.0/5.0, limiter.LimitJerk(vel, prevVel, prevPrevVel, dt));
  EXPECT_DOUBLE_EQ(4.0, vel);

  // Above upper acceleration bound (desired accel: 3.0)
  vel = 4.0;
  prevVel = 1.0;
  prevPrevVel = 0.0;
  EXPECT_DOUBLE_EQ(3.0/4.0, limiter.LimitJerk(vel, prevVel, prevPrevVel, dt));
  EXPECT_DOUBLE_EQ(3.0, vel);

  // Above upper jerk bound (desired jerk: 1.5, accel: 1.8)
  vel = 2.1;
  prevVel = 0.3;
  prevPrevVel = 0.0;
  EXPECT_DOUBLE_EQ(1.6/2.1, limiter.LimitJerk(vel, prevVel, prevPrevVel, dt));
  EXPECT_DOUBLE_EQ(1.6, vel);
}
