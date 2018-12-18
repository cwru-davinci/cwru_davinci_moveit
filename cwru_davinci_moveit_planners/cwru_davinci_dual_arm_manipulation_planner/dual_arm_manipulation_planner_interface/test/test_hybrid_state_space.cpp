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

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>
#include "Hybrid_State_Space_Test.h"


using namespace dual_arm_manipulation_planner_interface;
using namespace hybrid_state_space_test;
using namespace ompl::base;

//TEST(HybridStateSpace, hybridStateSpaceTest)
//{
////  ros::init(argc, argv, "davinci_simple_needle_grasp_main_node");
//  ros::NodeHandle node_handle;
//  ros::NodeHandle node_handle_priv("~");
//
//  cwru_davinci_grasp::DavinciSimpleNeedleGrasper simpleGrasp(node_handle,
//                                                             node_handle_priv,
//                                                             "needle_r", "psm_one");
//
//  std::vector<cwru_davinci_grasp::GraspInfo> grasp_pose = simpleGrasp.getAllPossibleNeedleGrasps();
//  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_pose.size() ,grasp_pose));
//
//  RealVectorBounds se3_xyz_bounds(3);
//  se3_xyz_bounds.setLow(0, -1.0);
//  se3_xyz_bounds.setHigh(0, 1.0);
//  se3_xyz_bounds.setLow(1, -1.0);
//  se3_xyz_bounds.setHigh(1, 1.0);
//  se3_xyz_bounds.setLow(2, -1.0);
//  se3_xyz_bounds.setHigh(2, 1.0);
//
//  hystsp->setSE3Bounds(se3_xyz_bounds);
////  base::DiscreteStateSpace &dm = *d;
//  hystsp->setup();
////  hystsp->sanityChecks();
//  HybridStateSpaceTest hystspt(hystsp, 1000, 1e-15);
//
//  ScopedState<HybridObjectStateSpace> s1(hystsp);
//  ScopedState<HybridObjectStateSpace> s2(hystsp);
//  ScopedState<HybridObjectStateSpace> cstate(hystsp);
//
//  s1.random();
//  s1->armIndex().value = 1;
//  s2->armIndex().value = 1;
//
//  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//  GTEST_ASSERT_EQ(cstate->armIndex().value, 2);
//};

void printGraspPart(int part)
{
  std::cout <<"Grasp Part ID " << part << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_hybrid_state_space");
  ros::AsyncSpinner spinner(1);
  ros::Duration(3.0).sleep();
  spinner.start();
//  testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();


  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");
  cwru_davinci_grasp::DavinciSimpleNeedleGrasper simpleGrasp(node_handle,
                                                             node_handle_priv,
                                                             "needle_r", "psm_one");

  std::vector<cwru_davinci_grasp::GraspInfo> grasp_pose = simpleGrasp.getAllPossibleNeedleGrasps();
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_pose.size() ,grasp_pose));

  RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -1.0);
  se3_xyz_bounds.setHigh(0, 1.0);
  se3_xyz_bounds.setLow(1, -1.0);
  se3_xyz_bounds.setHigh(1, 1.0);
  se3_xyz_bounds.setLow(2, -1.0);
  se3_xyz_bounds.setHigh(2, 1.0);

  hystsp->setSE3Bounds(se3_xyz_bounds);
//  base::DiscreteStateSpace &dm = *d;
  hystsp->setup();
//  hystsp->sanityChecks();
  HybridStateSpaceTest hystspt(hystsp, 1000, 1e-15);

  ScopedState<HybridObjectStateSpace> s1(hystsp);
  ScopedState<HybridObjectStateSpace> s2(hystsp);
  ScopedState<HybridObjectStateSpace> cstate(hystsp);

  // case StateDiff::AllSame

  std::cout << "!----------------------------------------------------!" << std::endl;
  std::cout << "case StateDiff::AllSame" << std::endl;
  s1.random();
  s2 = s1;

  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  hystsp->printState(s1.get(), std::cout);
  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);

  hystsp->printState(s2.get(), std::cout);
  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);

  hystsp->printState(cstate.get(), std::cout);
  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);

  // case StateDiff::ArmDiffGraspAndPoseSame

  std::cout << "!----------------------------------------------------!" << std::endl;
  std::cout << "case StateDiff::ArmDiffGraspAndPoseSame" << std::endl;
  s1.random();
  s2 = s1;

  s1->armIndex().value = 1;
  s2->armIndex().value = 2;

  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  hystsp->printState(s1.get(), std::cout);
  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);

  hystsp->printState(s2.get(), std::cout);
  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);

  hystsp->printState(cstate.get(), std::cout);
  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);


  // case StateDiff::GraspDiffArmAndPoseSame

  std::cout << "!----------------------------------------------------!" << std::endl;
  std::cout << "case StateDiff::GraspDiffArmAndPoseSame" << std::endl;
  s1.random();
  s2 = s1;

  s1->graspIndex().value = 8743;  // a random number
  s2->graspIndex().value = 4532;  // a random number

  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  hystsp->printState(s1.get(), std::cout);
  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);

  hystsp->printState(s2.get(), std::cout);
  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);

  hystsp->printState(cstate.get(), std::cout);
  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);

  // case StateDiff::PoseDiffArmAndGraspSame

  std::cout << "!----------------------------------------------------!" << std::endl;
  std::cout << "case StateDiff::PoseDiffArmAndGraspSame" << std::endl;

  s1.random();
  s2.random();

  s2->armIndex().value = s1->armIndex().value;
  s2->graspIndex().value = s1->graspIndex().value;

  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  hystsp->printState(s1.get(), std::cout);
  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);

  hystsp->printState(s2.get(), std::cout);
  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);

  hystsp->printState(cstate.get(), std::cout);
  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);


  // case StateDiff::ArmAndGraspDiffPoseSame

  std::cout << "!----------------------------------------------------!" << std::endl;
  std::cout << "case StateDiff::ArmAndGraspDiffPoseSame" << std::endl;

  s1.random();
  s2.random();

  hystsp->as<ompl::base::SE3StateSpace>(0)->copyState(&(s1->se3State()), &(s2->se3State()));

  s1->armIndex().value = 1;
  s2->armIndex().value = 2;


  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  hystsp->printState(s1.get(), std::cout);
  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);

  hystsp->printState(s2.get(), std::cout);
  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);

  hystsp->printState(cstate.get(), std::cout);
  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);


  // case StateDiff::ArmAndPoseDiffGraspSame

  std::cout << "!----------------------------------------------------!" << std::endl;
  std::cout << "case StateDiff::ArmAndPoseDiffGraspSame" << std::endl;

  s1.random();
  s2.random();

  s1->armIndex().value = 1;
  s2->armIndex().value = 2;

  s2->graspIndex().value = s1->graspIndex().value;

  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  hystsp->printState(s1.get(), std::cout);
  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);

  hystsp->printState(s2.get(), std::cout);
  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);

  hystsp->printState(cstate.get(), std::cout);
  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);


  // case StateDiff::GraspAndPoseDiffArmSame

  std::cout << "!----------------------------------------------------!" << std::endl;
  std::cout << "case StateDiff::GraspAndPoseDiffArmSame" << std::endl;

  s1.random();
  s2.random();

  s2->armIndex().value = s1->armIndex().value;

  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  hystsp->printState(s1.get(), std::cout);
  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);

  hystsp->printState(s2.get(), std::cout);
  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);

  hystsp->printState(cstate.get(), std::cout);
  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);


  // case StateDiff::AllDiff

  std::cout << "!----------------------------------------------------!" << std::endl;
  std::cout << "case StateDiff::AllDiff" << std::endl;
  s1.random();
  s2.random();

  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  hystsp->printState(s1.get(), std::cout);
  printGraspPart(grasp_pose[s1->graspIndex().value].part_id);

  hystsp->printState(s2.get(), std::cout);
  printGraspPart(grasp_pose[s2->graspIndex().value].part_id);

  hystsp->printState(cstate.get(), std::cout);
  printGraspPart(grasp_pose[cstate->graspIndex().value].part_id);


//  hystsp->freeState(s1.get());
//  hystsp->freeState(s2.get());
//  hystsp->freeState(cstate.get());
//
  ros::Duration(3.0).sleep();
  ros::shutdown();
  return 0;
}
