/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Case Western Reserve University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Su Lu */


#include <ros/ros.h>
#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"
//#include "Hybrid_State_Space_Test.h"

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>

#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>

using namespace std;
using namespace std::chrono;
using namespace std::this_thread;


using namespace dual_arm_manipulation_planner_interface;
// using namespace hybrid_state_space_test;
namespace ob =  ompl::base;

void chooseGraspIndex(const std::vector<cwru_davinci_grasp::GraspInfo>& graspIndex,
                      const int from_grasp_index,
                      int& to_grasp_index,
                      bool same_grasp_part)
{
  if(same_grasp_part)
  {
    for(std::size_t i = 0; i < graspIndex.size(); ++i)
    {
      if(graspIndex[i].part_id == graspIndex[from_grasp_index].part_id && i != from_grasp_index)
      {
        to_grasp_index = i;
        return;
      }
    }
  }
  else
  {
    for(std::size_t i = 0; i < graspIndex.size(); ++i)
    {
      if(graspIndex[i].part_id != graspIndex[from_grasp_index].part_id && i != from_grasp_index)
      {
        to_grasp_index = i;
        return;
      }
    }
  }
}


TEST(TestHybridRRT, HybridObjectStateSpace)
{
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");
  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr pSimpleGrasp(
        new cwru_davinci_grasp::DavinciSimpleNeedleGrasper(node_handle,
                                                           node_handle_priv,
                                                           "psm_one",
                                                           "needle_r"));
//  EXPECT_TRUE(pSimpleGrasp);

  std::vector<cwru_davinci_grasp::GraspInfo> grasp_pose = pSimpleGrasp->getAllPossibleNeedleGrasps();
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_pose.size(), grasp_pose));

  ob::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.101);
  se3_xyz_bounds.setHigh(0, 0.101);
  se3_xyz_bounds.setLow(1, -0.06);
  se3_xyz_bounds.setHigh(1, 0.09);
  se3_xyz_bounds.setLow(2, 0.266);
  se3_xyz_bounds.setHigh(2, 0.496);

  hystsp->setSE3Bounds(se3_xyz_bounds);
  hystsp->setup();
  
  // test checkStateDiff and distance
  {
    ob::ScopedState<HybridObjectStateSpace> s1(hystsp);
    ob::ScopedState<HybridObjectStateSpace> s2(hystsp);

    // test checkStateDiff
    // case StateDiff::AllSame
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s1->armIndex().value = 1;
    s1->graspIndex().value = 1;

    s2->se3State().setX(1.0); s2->se3State().setY(0.0); s2->se3State().setZ(0.0);
    s2->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s2->armIndex().value = 1;
    s2->graspIndex().value = 1;
    EXPECT_EQ(StateDiff::AllSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    EXPECT_EQ(0.0, hystsp->distance(s1.get(), s2.get()));

    // case StateDiff::ArmDiffGraspAndPoseSame
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s1->graspIndex().value = 1;

    s2->se3State().setX(1.0); s2->se3State().setY(0.0); s2->se3State().setZ(0.0);
    s2->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s2->graspIndex().value = 1;

    s1->armIndex().value = 1;
    s2->armIndex().value = 2;
    EXPECT_EQ(StateDiff::ArmDiffGraspAndPoseSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    EXPECT_EQ(300, hystsp->distance(s1.get(), s2.get()));

    // case StateDiff::GraspDiffArmAndPoseSame
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s1->armIndex().value = 1;

    s2->se3State().setX(1.0); s2->se3State().setY(0.0); s2->se3State().setZ(0.0);
    s2->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s2->armIndex().value = 1;

    s1->graspIndex().value = 10;  // a random number
    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, false);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[s2->graspIndex().value].part_id);

    EXPECT_EQ(StateDiff::GraspDiffArmAndPoseSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    EXPECT_EQ(200, hystsp->distance(s1.get(), s2.get()));

    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, true);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[s2->graspIndex().value].part_id);

    EXPECT_EQ(StateDiff::GraspDiffArmAndPoseSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    EXPECT_EQ(200, hystsp->distance(s1.get(), s2.get()));

    // case StateDiff::PoseDiffArmAndGraspSame
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s1->armIndex().value = 1;
    s1->graspIndex().value = 1;

    s2->se3State().setX(1.0); s2->se3State().setY(0.0); s2->se3State().setZ(0.0);
    s2->se3State().rotation().setAxisAngle(0.0, 1.0, 0.0, M_PI);
    s2->armIndex().value = 1;
    s2->graspIndex().value = 1;
    EXPECT_EQ(StateDiff::PoseDiffArmAndGraspSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    double object_se3_dist = hystsp->getSubspace(0)->distance(s1->components[0], s2->components[0]);
    double handoff_dist = 0;
    double total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(object_se3_dist, hystsp->distance(s1.get(), s2.get()));

    // case StateDiff::ArmAndGraspDiffPoseSame
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s1->armIndex().value = 1;
    s1->graspIndex().value = 10;

    s2->se3State().setX(1.0); s2->se3State().setY(0.0); s2->se3State().setZ(0.0);
    s2->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s2->armIndex().value = 2;
    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, false);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[s2->graspIndex().value].part_id);

    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    object_se3_dist = hystsp->getSubspace(0)->distance(s1->components[0], s2->components[0]);
    handoff_dist = 100;
    total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));

    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, true);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[s2->graspIndex().value].part_id);
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    handoff_dist = 300;
    total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));

    // case StateDiff::ArmAndPoseDiffGraspSame
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s1->armIndex().value = 1;
    s1->graspIndex().value = 10;

    s2->se3State().setX(1.0); s2->se3State().setY(0.0); s2->se3State().setZ(0.0);
    s2->se3State().rotation().setAxisAngle(0.0, 1.0, 0.0, M_PI);
    s2->armIndex().value = 2;
    s2->graspIndex().value = 10;
    EXPECT_EQ(StateDiff::ArmAndPoseDiffGraspSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    object_se3_dist = hystsp->getSubspace(0)->distance(s1->components[0], s2->components[0]);
    handoff_dist = 300;
    total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));

    // case StateDiff::GraspAndPoseDiffArmSame
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s1->armIndex().value = 1;
    s1->graspIndex().value = 10;

    s2->se3State().setX(1.0); s2->se3State().setY(0.0); s2->se3State().setZ(0.0);
    s2->se3State().rotation().setAxisAngle(0.0, 1.0, 0.0, M_PI);
    s2->armIndex().value = 1;
    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, false);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[s2->graspIndex().value].part_id);
    EXPECT_EQ(StateDiff::GraspAndPoseDiffArmSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    object_se3_dist = hystsp->getSubspace(0)->distance(s1->components[0], s2->components[0]);
    handoff_dist = 200;
    total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));

    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, true);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[s2->graspIndex().value].part_id);
    EXPECT_EQ(StateDiff::GraspAndPoseDiffArmSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));

    // case StateDiff::AllDiff
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, M_PI);
    s1->armIndex().value = 1;
    s1->graspIndex().value = 10;

    s2->se3State().setX(1.0); s2->se3State().setY(0.0); s2->se3State().setZ(0.0);
    s2->se3State().rotation().setAxisAngle(0.0, 1.0, 0.0, M_PI);
    s2->armIndex().value = 2;
    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, false);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[s2->graspIndex().value].part_id);
    EXPECT_EQ(StateDiff::AllDiff, hystsp->checkStateDiff(s1.get(), s2.get()));
    object_se3_dist = hystsp->getSubspace(0)->distance(s1->components[0], s2->components[0]);
    handoff_dist = 100;
    total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));

    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, true);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[s2->graspIndex().value].part_id);
    EXPECT_EQ(StateDiff::AllDiff, hystsp->checkStateDiff(s1.get(), s2.get()));
    handoff_dist = 300;
    total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));
  }

  // test copyToReals
  {
    ob::ScopedState<HybridObjectStateSpace> s1(hystsp);
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, 0.0);
    s1->armIndex().value = 1;
    s1->graspIndex().value = 10;

    std::vector<double> s1Vec;
    hystsp->copyToReals(s1Vec, s1.get());
    EXPECT_EQ(1.0, s1Vec[6]);
  }

  // test interpolate
  {
    ob::ScopedState<HybridObjectStateSpace> from(hystsp);
    ob::ScopedState<HybridObjectStateSpace> to(hystsp);

    from->se3State().setX(0.0891485);
    from->se3State().setY(-0.0156728);
    from->se3State().setZ(0.308404);
    from->se3State().rotation().x = 0.0750505;
    from->se3State().rotation().y = 0.0357382;
    from->se3State().rotation().z = 0.91119;
    from->se3State().rotation().w = 0.403514;
    from->armIndex().value = 1;
    from->graspIndex().value = 8;

    to->se3State().setX(0.00540861);
    to->se3State().setY(-0.00887215);
    to->se3State().setZ(0.39268);
    to->se3State().rotation().x = -0.0715043;
    to->se3State().rotation().y = -0.0940231;
    to->se3State().rotation().z = 0.775785;
    to->se3State().rotation().w = 0.619842;
    to->armIndex().value = 1;
    to->graspIndex().value = 8;
    ompl::base::SO3StateSpace* pSO3StSp = dynamic_cast<ob::SO3StateSpace*>(
      dynamic_cast<ob::SE3StateSpace*>(hystsp->getSubspace(0).get())->getSubspace(1).get());

    EXPECT_TRUE(pSO3StSp);
    pSO3StSp->enforceBounds(&from->se3State().rotation());
    pSO3StSp->enforceBounds(&to->se3State().rotation());

    std::vector<double> fromVec;
    std::vector<double> toVec;
    hystsp->copyToReals(fromVec, from.get());
    hystsp->copyToReals(toVec, to.get());

    EXPECT_EQ(StateDiff::PoseDiffArmAndGraspSame, hystsp->checkStateDiff(from.get(), to.get()));
    double se3_dist = hystsp->getSubspace(0)->distance(&from->se3State(), &to->se3State());
    double hybrid_dist = hystsp->distance(from.get(), to.get());
    EXPECT_EQ(se3_dist, hybrid_dist);
    double max_distance = 100;
    EXPECT_LT(hybrid_dist, max_distance);

    ob::ScopedState<HybridObjectStateSpace> hybridCState(hystsp);
    ob::ScopedState<ob::SE3StateSpace> se3CState(hystsp->getSubspace(0));
    hystsp->interpolate(from.get(), to.get(), se3_dist/max_distance, hybridCState.get());
    hystsp->getSubspace(0)->interpolate(&from->se3State(), &to->se3State(), se3_dist/max_distance, se3CState.get());
    EXPECT_TRUE(hystsp->getSubspace(0)->equalStates(&hybridCState->se3State(), se3CState.get()));
  }


//
//
//
//
//
//
//
//  // case StateDiff::AllSame
//
//  std::cout << "!----------------------------------------------------!" << std::endl;
//  std::cout << "case StateDiff::AllSame" << std::endl;
//  s1.random();
//  s2 = s1;
//
//  double distance = hystsp->distance(s1.get(), s2.get());
//  EXPECT_EQ(0, distance);
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//
//  hystsp->printState(s1.get(), std::cout);
//  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);
//
//  hystsp->printState(s2.get(), std::cout);
//  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);
//
//  hystsp->printState(cstate.get(), std::cout);
//  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);
//
//  // case StateDiff::ArmDiffGraspAndPoseSame
//
//  std::cout << "!----------------------------------------------------!" << std::endl;
//  std::cout << "case StateDiff::ArmDiffGraspAndPoseSame" << std::endl;
//  s1.random();
//  s2 = s1;
//
//  s1->armIndex().value = 1;
//  s2->armIndex().value = 2;
//  distance = hystsp->distance(s1.get(), s2.get());
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//
//  hystsp->printState(s1.get(), std::cout);
//  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);
//
//  hystsp->printState(s2.get(), std::cout);
//  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);
//
//  hystsp->printState(cstate.get(), std::cout);
//  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);
//
//
//  // case StateDiff::GraspDiffArmAndPoseSame
//
//  std::cout << "!----------------------------------------------------!" << std::endl;
//  std::cout << "case StateDiff::GraspDiffArmAndPoseSame" << std::endl;
//  s1.random();
//  s2 = s1;
//
//  s1->graspIndex().value = 8743;  // a random number
//  s2->graspIndex().value = 4532;  // a random number
//  distance = hystsp->distance(s1.get(), s2.get());
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//
//  hystsp->printState(s1.get(), std::cout);
//  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);
//
//  hystsp->printState(s2.get(), std::cout);
//  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);
//
//  hystsp->printState(cstate.get(), std::cout);
//  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);
//
//  // case StateDiff::PoseDiffArmAndGraspSame
//
//  std::cout << "!----------------------------------------------------!" << std::endl;
//  std::cout << "case StateDiff::PoseDiffArmAndGraspSame" << std::endl;
//
//  s1.random();
//  s2.random();
//
//  s2->armIndex().value = s1->armIndex().value;
//  s2->graspIndex().value = s1->graspIndex().value;
//  distance = hystsp->distance(s1.get(), s2.get());
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//
//  hystsp->printState(s1.get(), std::cout);
//  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);
//
//  hystsp->printState(s2.get(), std::cout);
//  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);
//
//  hystsp->printState(cstate.get(), std::cout);
//  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);
//
//
//  // case StateDiff::ArmAndGraspDiffPoseSame
//
//  std::cout << "!----------------------------------------------------!" << std::endl;
//  std::cout << "case StateDiff::ArmAndGraspDiffPoseSame" << std::endl;
//
//  s1.random();
//  s2.random();
//
//  hystsp->as<ob::SE3StateSpace>(0)->copyState(&(s1->se3State()), &(s2->se3State()));
//
//  s1->armIndex().value = 1;
//  s2->armIndex().value = 2;
//  distance = hystsp->distance(s1.get(), s2.get());
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//
//  hystsp->printState(s1.get(), std::cout);
//  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);
//
//  hystsp->printState(s2.get(), std::cout);
//  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);
//
//  hystsp->printState(cstate.get(), std::cout);
//  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);
//
//
//  // case StateDiff::ArmAndPoseDiffGraspSame
//
//  std::cout << "!----------------------------------------------------!" << std::endl;
//  std::cout << "case StateDiff::ArmAndPoseDiffGraspSame" << std::endl;
//
//  s1.random();
//  s2.random();
//
//  s1->armIndex().value = 1;
//  s2->armIndex().value = 2;
//
//  s2->graspIndex().value = s1->graspIndex().value;
//  distance = hystsp->distance(s1.get(), s2.get());
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//
//  hystsp->printState(s1.get(), std::cout);
//  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);
//
//  hystsp->printState(s2.get(), std::cout);
//  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);
//
//  hystsp->printState(cstate.get(), std::cout);
//  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);
//
//
//  // case StateDiff::GraspAndPoseDiffArmSame
//
//  std::cout << "!----------------------------------------------------!" << std::endl;
//  std::cout << "case StateDiff::GraspAndPoseDiffArmSame" << std::endl;
//
//  s1.random();
//  s2.random();
//
//  s2->armIndex().value = s1->armIndex().value;
//  distance = hystsp->distance(s1.get(), s2.get());
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//
//  hystsp->printState(s1.get(), std::cout);
//  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);
//
//  hystsp->printState(s2.get(), std::cout);
//  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);
//
//  hystsp->printState(cstate.get(), std::cout);
//  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);
//
//
//  // case StateDiff::AllDiff
//
//  std::cout << "!----------------------------------------------------!" << std::endl;
//  std::cout << "case StateDiff::AllDiff" << std::endl;
//  s1.random();
//  s2.random();
//  distance = hystsp->distance(s1.get(), s2.get());
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//
//  hystsp->printState(s1.get(), std::cout);
//  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);
//
//  hystsp->printState(s2.get(), std::cout);
//  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);
//
//  hystsp->printState(cstate.get(), std::cout);
//  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_hybrid_state_space");
  ros::NodeHandle nh;
  ros::Duration(3.0).sleep();
  return RUN_ALL_TESTS();
}
