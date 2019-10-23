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
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <cwru_davinci_grasp/davinci_needle_grasper_base.h>

#include <gtest/gtest.h>
#include <math.h>

namespace ob =  ompl::base;
using namespace dual_arm_manipulation_planner_interface;

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
  ros::NodeHandle node_handle_priv("~");
  cwru_davinci_grasp::DavinciNeedleGrasperBasePtr pSimpleGrasp(
    new cwru_davinci_grasp::DavinciNeedleGrasperBase(node_handle_priv,
                                                     "psm_one",
                                                     "psm_one_gripper"));
//  EXPECT_TRUE(pSimpleGrasp);

  std::vector<cwru_davinci_grasp::GraspInfo> grasp_pose = pSimpleGrasp->getAllPossibleNeedleGrasps();
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_pose.size()-1, grasp_pose));

  ob::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.101);
  se3_xyz_bounds.setHigh(0, 0.101);
  se3_xyz_bounds.setLow(1, -0.06);
  se3_xyz_bounds.setHigh(1, 0.09);
  se3_xyz_bounds.setLow(2, 0.266);
  se3_xyz_bounds.setHigh(2, 0.496);

  hystsp->setSE3Bounds(se3_xyz_bounds);
  hystsp->setup();

  // construct an instance of space information from hybrid object state space
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  // test HybridObjectStateSpace::StateType
  {
    ob::ScopedState<HybridObjectStateSpace> hyState(hystsp);
    EXPECT_EQ(0, hyState->flags);
    EXPECT_TRUE(!hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isMarkedValid());
    hyState->markInvalid();
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());
    hyState->setJointsComputed(false);
    EXPECT_TRUE(!hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    hyState->setJointsComputed(true);
    EXPECT_TRUE(hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    hyState->markValid();
    EXPECT_TRUE(hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(hyState->jointsComputed());
    hyState->setJointsComputed(true);
    EXPECT_TRUE(hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(hyState->jointsComputed());
    hyState->setJointsComputed(false);
    EXPECT_TRUE(hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());
    hyState->markInvalid();
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());

    hyState->clearKnownInformation();
    EXPECT_EQ(0, hyState->flags);
    EXPECT_TRUE(!hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isMarkedValid());
    hyState->markValid();
    EXPECT_TRUE(hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());
    hyState->setJointsComputed(true);
    EXPECT_TRUE(hyState->jointsComputed());
    EXPECT_TRUE(hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    hyState->setJointsComputed(false);
    EXPECT_TRUE(!hyState->jointsComputed());
    EXPECT_TRUE(hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    hyState->markInvalid();
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());
    hyState->setJointsComputed(false);
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());
    hyState->setJointsComputed(true);
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(hyState->jointsComputed());
    hyState->markValid();
    EXPECT_TRUE(hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(hyState->jointsComputed());

    hyState->clearKnownInformation();
    EXPECT_EQ(0, hyState->flags);
    EXPECT_TRUE(!hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isMarkedValid());
    hyState->setJointsComputed(true);
    EXPECT_TRUE(hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->isMarkedValid());
    hyState->setJointsComputed(false);
    EXPECT_TRUE(!hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->isMarkedValid());
    hyState->setJointsComputed(true);
    EXPECT_TRUE(hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->isMarkedValid());
    hyState->markValid();
    EXPECT_TRUE(hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(hyState->jointsComputed());
    hyState->markInvalid();
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(hyState->jointsComputed());
    hyState->setJointsComputed(false);
    EXPECT_TRUE(!hyState->jointsComputed());
    EXPECT_TRUE(!hyState->isMarkedValid());
    EXPECT_TRUE(hyState->isValidityKnown());
    hyState->setJointsComputed(true);
    EXPECT_TRUE(hyState->jointsComputed());
    EXPECT_TRUE(hyState->isValidityKnown());
    EXPECT_TRUE(!hyState->isMarkedValid());

    ob::State* state = si->allocState();
    HybridObjectStateSpace::StateType* pHyState = state->as<HybridObjectStateSpace::StateType>();
    EXPECT_TRUE(pHyState);
    EXPECT_EQ(0, pHyState->flags);
  }

  // test checkStateDiff and distance
  {
    ob::ScopedState<HybridObjectStateSpace> s1(hystsp);
    ob::ScopedState<HybridObjectStateSpace> s2(hystsp);
    ob::ScopedState<HybridObjectStateSpace> cs(hystsp);

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
    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::AllSame, hystsp->checkStateDiff(s1.get(), cs.get()));

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
    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

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
    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, true);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[s2->graspIndex().value].part_id);

    EXPECT_EQ(StateDiff::GraspDiffArmAndPoseSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    EXPECT_EQ(200, hystsp->distance(s1.get(), s2.get()));
    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

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
    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::PoseDiffArmAndGraspSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value == cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value == cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[cs->graspIndex().value].part_id);

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
    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, true);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[s2->graspIndex().value].part_id);
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    handoff_dist = 300;
    total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));
    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

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

    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

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

    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, true);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[s2->graspIndex().value].part_id);
    EXPECT_EQ(StateDiff::GraspAndPoseDiffArmSame, hystsp->checkStateDiff(s1.get(), s2.get()));
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));

    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

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

    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);

    chooseGraspIndex(grasp_pose, s1->graspIndex().value, s2->graspIndex().value, true);
    EXPECT_TRUE(s1->graspIndex().value != s2->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id == grasp_pose[s2->graspIndex().value].part_id);
    EXPECT_EQ(StateDiff::AllDiff, hystsp->checkStateDiff(s1.get(), s2.get()));
    handoff_dist = 300;
    total_dist = object_se3_dist + handoff_dist;
    EXPECT_EQ(total_dist, hystsp->distance(s1.get(), s2.get()));

    hystsp->interpolate(s1.get(), s2.get(), 0.5, cs.get());
    EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, hystsp->checkStateDiff(s1.get(), cs.get()));
    EXPECT_TRUE(s1->armIndex().value != cs->armIndex().value);
    EXPECT_TRUE(s1->graspIndex().value != cs->graspIndex().value);
    EXPECT_TRUE(grasp_pose[s1->graspIndex().value].part_id != grasp_pose[cs->graspIndex().value].part_id);
  }

  // test copyToReals
  {
    ob::ScopedState<HybridObjectStateSpace> s1(hystsp);
    s1->se3State().setX(1.0); s1->se3State().setY(0.0); s1->se3State().setZ(0.0);
    s1->se3State().rotation().setAxisAngle(1.0, 0.0, 0.0, 0.0);
    s1->armIndex().value = 2;
    s1->graspIndex().value = 10;

    std::vector<double> s1Vec;
    hystsp->copyToReals(s1Vec, s1.get());
    EXPECT_EQ(9, s1Vec.size());

    // test SO3 part
    EXPECT_EQ(0.0, s1Vec[3]);
    EXPECT_EQ(0.0, s1Vec[4]);
    EXPECT_EQ(0.0, s1Vec[5]);
    EXPECT_EQ(1.0, s1Vec[6]);

    // test arm index and grasp index
    EXPECT_EQ(2.0, s1Vec[7]);
    EXPECT_EQ(10.0, s1Vec[8]);
  }

  // test se3ToEigen3d, eigen3dToSE3
  {
    ob::ScopedState<HybridObjectStateSpace> rdmHYSE3(hystsp);
    rdmHYSE3.random();
    Eigen::Affine3d eigenSE3;
    hystsp->se3ToEigen3d(rdmHYSE3.get(), eigenSE3);

    ob::ScopedState<HybridObjectStateSpace> copyHYSE3(hystsp);
    hystsp->eigen3dToSE3(eigenSE3, copyHYSE3.get());

    // test equalStates
    EXPECT_TRUE(hystsp->getSubspace(0)->equalStates(&rdmHYSE3->se3State(), &copyHYSE3->se3State()));
    EXPECT_EQ(rdmHYSE3->se3State().getX(), copyHYSE3->se3State().getX());
    EXPECT_EQ(rdmHYSE3->se3State().getY(), copyHYSE3->se3State().getY());
    EXPECT_EQ(rdmHYSE3->se3State().getZ(), copyHYSE3->se3State().getZ());
    EXPECT_NEAR(rdmHYSE3->se3State().rotation().w, copyHYSE3->se3State().rotation().w, 1e-6);
    EXPECT_NEAR(rdmHYSE3->se3State().rotation().x, copyHYSE3->se3State().rotation().x, 1e-6);
    EXPECT_NEAR(rdmHYSE3->se3State().rotation().y, copyHYSE3->se3State().rotation().y, 1e-6);
    EXPECT_NEAR(rdmHYSE3->se3State().rotation().z, copyHYSE3->se3State().rotation().z, 1e-6);

    Eigen::Affine3d indentityEigenSE3(Eigen::Affine3d::Identity());
    ob::ScopedState<HybridObjectStateSpace> indentityHYSE3(hystsp);
    hystsp->eigen3dToSE3(indentityEigenSE3, indentityHYSE3.get());

    EXPECT_EQ(0.0, indentityHYSE3->se3State().getX());
    EXPECT_EQ(0.0, indentityHYSE3->se3State().getY());
    EXPECT_EQ(0.0, indentityHYSE3->se3State().getZ());
    EXPECT_EQ(1.0, indentityHYSE3->se3State().rotation().w);
    EXPECT_EQ(0.0, indentityHYSE3->se3State().rotation().x);
    EXPECT_EQ(0.0, indentityHYSE3->se3State().rotation().y);
    EXPECT_EQ(0.0, indentityHYSE3->se3State().rotation().z);

    const double u1 = Eigen::internal::random<double>(0, 1),
                 u2 = Eigen::internal::random<double>(0, 2*EIGEN_PI),
                 u3 = Eigen::internal::random<double>(0, 2*EIGEN_PI);
    const double a = sqrt(double(1) - u1),
                 b = sqrt(u1);
    Eigen::Quaterniond rdmQua(a * sin(u2), a * cos(u2), b * sin(u3), b * cos(u3));

    Eigen::Affine3d rdmAffine3d;
    rdmAffine3d.translation() << 1.0, 1.2, 1.1;
    rdmAffine3d.linear() = rdmQua.toRotationMatrix();

    hystsp->eigen3dToSE3(rdmAffine3d, rdmHYSE3.get());
    EXPECT_EQ(rdmHYSE3->se3State().getX(), 1.0);
    EXPECT_EQ(rdmHYSE3->se3State().getY(), 1.2);
    EXPECT_EQ(rdmHYSE3->se3State().getZ(), 1.1);
    EXPECT_NEAR(fabs(rdmHYSE3->se3State().rotation().w), fabs(rdmQua.w()), 1e-6);
    EXPECT_NEAR(fabs(rdmHYSE3->se3State().rotation().x), fabs(rdmQua.x()), 1e-6);
    EXPECT_NEAR(fabs(rdmHYSE3->se3State().rotation().y), fabs(rdmQua.y()), 1e-6);
    EXPECT_NEAR(fabs(rdmHYSE3->se3State().rotation().z), fabs(rdmQua.z()), 1e-6);

    Eigen::Affine3d copyAffine3d;
    hystsp->se3ToEigen3d(rdmHYSE3.get(), copyAffine3d);
    EXPECT_TRUE(copyAffine3d.isApprox(rdmAffine3d, 1e-6));
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

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_hybrid_state_space");
  ros::NodeHandle nh;
  ros::Duration(3.0).sleep();
  return RUN_ALL_TESTS();
}
