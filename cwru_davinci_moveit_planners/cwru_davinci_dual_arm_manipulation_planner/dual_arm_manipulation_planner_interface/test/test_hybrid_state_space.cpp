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


#include "ompl/base/ScopedState.h"
#include "ompl/base/SpaceInformation.h"

//#include <boost/math/constants/constants.hpp>
#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
//#include <pluginlib/class_loader.h>

#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>
#include "Hybrid_State_Space_Test.h"
// MoveIt!
//#include <moveit/robot_model_loader/robot_model_loader.h>
//#include <moveit/robot_model/robot_model.h>
//#include <moveit/robot_state/robot_state.h>
//#include <moveit/rdf_loader/rdf_loader.h>

//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <urdf_interface/model.h>
//#include <urdf/model.h>
//#include <srdfdom/model.h>

#include <fstream>
#include <iostream>
#include <cstdlib> // for exit()

using namespace dual_arm_manipulation_planner_interface;
using namespace hybrid_state_space_test;
using namespace ompl::base;

TEST(HybridStateSpace, hybridStateSpaceTest)
{
//  ros::init(argc, argv, "davinci_simple_needle_grasp_main_node");
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

  s1.random();
  s1->armIndex().value = 1;
  s2->armIndex().value = 1;

  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
  GTEST_ASSERT_EQ(cstate->armIndex().value, 2);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_hybrid_state_space");
  ros::AsyncSpinner spinner(1);
  ros::Duration(3.0).sleep();
  spinner.start();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
