/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include <thread>
#include <gtest/gtest.h>

#include "ignition/math/Helpers.hh"
#include "ignition/math/FrameGraph.hh"

using namespace ignition;
using namespace math;

/////////////////////////////////////////////////
TEST(FrameGraphTest, AbsolutePaths)
{
  // store result of various calls
  bool r;

  // frameGraph comes with a built-in "world" frame
  FrameGraph frameGraph;

  Pose3d pa(1, 0, 0, 0, 0, 0);

  // this path's parent is incorrect ("world has no /")
  EXPECT_THROW(frameGraph.AddFrame("world", "x", pa), FrameException);

  // this path is not fully qualified
  EXPECT_THROW(frameGraph.AddFrame("/world/..", "x", pa), FrameException);

  // this path as an undefined "unknown" frame
  EXPECT_THROW(frameGraph.AddFrame("/world/unknown", "x", pa), FrameException);

  // this path as an illegal "!" frame
  EXPECT_THROW(frameGraph.AddFrame("/world/!", "x", pa), FrameException);

  // very stupid attempt at getting pose info from inexistant frame
  Pose3d p;
  EXPECT_THROW(p = frameGraph.Pose("/world/x", "/world"), FrameException);
  EXPECT_THROW(p = frameGraph.Pose("/world", "/world/x"), FrameException);

  // Finally, this path adds a to the built in "/world" frame
  frameGraph.AddFrame("/world", "a", pa);
  Pose3d a2w;
  // skillfull pose inquiry
  a2w = frameGraph.Pose("/world/a", "/world");
  EXPECT_EQ(pa, a2w);

  // error: x does not exist
  EXPECT_THROW(p = frameGraph.Pose("/world/a", "/world/x"), FrameException);

  // add b
  Pose3d pb(0, 1, 0, 0, 0, 0);
  frameGraph.AddFrame("/world", "b", pb);

  // Tests using relative paths
  Pose3d w2b = frameGraph.Pose("/world/b", "..");
  EXPECT_EQ(pb, w2b);


  // Relative path from a to b
  Pose3d b2a, b2a2;
  b2a = frameGraph.Pose("/world/a", "/world/b");
  b2a2 = frameGraph.Pose("/world/a", "../b");
  EXPECT_EQ(b2a, b2a2);
}

/////////////////////////////////////////////////
TEST(FrameGraphTest, RelativePaths)
{
  //             world
  //              |
  //              a
  //              |
  //          --------
  //          |      |
  //          aa     ab
  //          |
  //         aaa

  Pose3d pa   ( 1,  0,  0,  0,  0,  0);
  Pose3d paa  ( 0,  1,  0,  0,  0,  0);
  Pose3d pab  ( -1, 0,  0,  0,  0,  0);
  Pose3d paaa ( 0,  0,  0,  0,  1.570790, 0);

  FrameGraph frameGraph;

  frameGraph.AddFrame("/world", "a", pa);
  frameGraph.AddFrame("/world/a", "aa", paa);
  frameGraph.AddFrame("/world/a/aa", "aaa", paaa);
  frameGraph.AddFrame("/world/a", "ab", pab);

  // try to add duplicate frame
  EXPECT_THROW(frameGraph.AddFrame("/world", "a",  pa), FrameException);
  std::cout << frameGraph.Pose("/world/a/aa/aaa", "/world") << std::endl;

  const auto &faa = frameGraph.FrameAccess("/world/a/aa");
  RelativePose rel = frameGraph.RelativePoses("/world/a/aa/aaa", "/world");

  double sweep = 2 * 3.1415926;
  // varie the local pose of aa, this should move aaa in the world
  unsigned int steps = 10;
  for(unsigned int i=0; i < (steps + 1); ++i)
  {
    Pose3d p (0, 1, 0, i * (sweep / steps), 0, 0);
    frameGraph.SetPose(faa, p);
    std::cout << "aa frame: " << p << std::endl;
    std::cout << "fg pose: " << frameGraph.Pose("/world/a/aa/aaa", "/world") << std::endl;
    std::cout << "rel pose: " << frameGraph.Pose(rel) << std::endl;
  }

  Pose3d p0;
  EXPECT_EQ(p0, frameGraph.Pose("/world/a/ab", "/world"));
}

/////////////////////////////////////////////////
TEST(FrameGraphTest, SimplePose)
{
  // In a graph with a single frame, the pose of
  // the frame should be the same as the relative
  // pose between the frame and the world
  FrameGraph frameGraph;

  Pose3d pa(1, 0, 0, 0, 0, 0);
  frameGraph.AddFrame("/world", "a", pa);

  Pose3d r;
  r = frameGraph.Pose("/world/a", "/world");
  EXPECT_EQ(pa, r);

  const auto &frame = frameGraph.FrameAccess("/world/a");
  EXPECT_EQ(pa, frameGraph.Pose(frame));

  Pose3d pb(2,0,0,0,0,0);
  frameGraph.SetPose(frame, pb);

  EXPECT_EQ(pb, frameGraph.Pose("/world/a", "/world"));
}

/////////////////////////////////////////////////
void asyncStuff(FrameGraph &frameGraph)
{
  const auto &frame = frameGraph.FrameAccess("/world/a");
  std::cout << "asyncStuff" << std::endl;
  for (int i=0; i < 10001; ++i)
  {
    Pose3d p(i, 0, 0, 0, 0, 0);
    frameGraph.SetPose(frame, p);
    // std::cout << "x" << i << std::endl;
  }
}

/////////////////////////////////////////////////
TEST(FrameGraphTest, Multithreads)
{
  // In a graph with a single frame, the pose of
  // the frame should be the same as the relative
  // pose between the frame and the world
  FrameGraph frameGraph;

  Pose3d pa(0, 0, 0, 0, 0, 0);
  frameGraph.AddFrame("/world", "a", pa);

  Pose3d r;
  r = frameGraph.Pose("/world/a", "/world");
  EXPECT_EQ(pa, r);

  std::vector<std::thread> pool;
  for (int i = 0; i < 1; ++i)
  {
    // std::thread t1{asyncStuff, std::ref(frameGraph)};
    pool.push_back(std::thread {asyncStuff, std::ref(frameGraph)});
  }
  const auto &frame = frameGraph.FrameAccess("/world/a");
  EXPECT_EQ(pa, frameGraph.Pose(frame));

  // sample the frame position
  //Pose3d p(1,0,0,0,0,0);
  //frameGraph.Pose("/world/a", "/world");
  //EXPECT_EQ(pa, p);
//  Pose3d last = p;

  // Pose3d p;

  auto rel = frameGraph.RelativePoses("/world/a", "/world");
  Pose3d last = frameGraph.Pose(rel);
  for (int i=0; i < 1000; ++i)
  {
    Pose3d p;
    p = frameGraph.Pose(rel);
    EXPECT_GE(p.Pos().X(), last.Pos().X());
    last = p;
  }

  for (auto &thread: pool)
  {
    thread.join();
  }
  Pose3d p = frameGraph.Pose(rel);
  EXPECT_EQ(p, frameGraph.Pose("/world/a", "/world"));
}

/////////////////////////////////////////////////
/*
TEST(FrameGraphTest, SandBox)
{
  std::cout << "===== X =====" << std::endl;

  class A
  {
    public:
      A(){ std::cout << "A" << std::endl;}
      ~A(){std::cout << "~A" << std::endl; }
  };

  A a;
  {
    A &b = a;
  }

}
*/
