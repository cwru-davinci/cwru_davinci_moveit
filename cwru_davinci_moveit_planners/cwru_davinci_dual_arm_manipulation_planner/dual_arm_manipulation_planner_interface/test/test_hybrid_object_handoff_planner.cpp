/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Case Western Reserve University
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
#include <cwru_davinci_grasp/davinci_needle_grasper_base.h>
#include "hybrid_object_handoff_planner_tester.cpp"

using namespace dual_arm_manipulation_planner_interface;
using namespace hybrid_planner_test;
namespace ob = ompl::base;

TEST(TestHybridRRT, HybridObjectHandoffPlanner)
{
  ros::NodeHandle nodeHandlePriv("~");
  robot_model_loader::RobotModelLoader robotModelLoader("robot_description");

  cwru_davinci_grasp::DavinciNeedleGrasperBasePtr simpleGrasp =
  boost::make_shared<cwru_davinci_grasp::DavinciNeedleGrasperBase>(nodeHandlePriv,
                                                                   "psm_one",
                                                                   "psm_one_gripper");


  HybridObjectHandoffPlannerTester tester(simpleGrasp->getAllPossibleNeedleGrasps(false),
                                          robotModelLoader.getModel());
  tester.testConnectStates();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_hybrid_object_handoff_planner");
  ros::NodeHandle nh;
  ros::Duration(3.0).sleep();
  return RUN_ALL_TESTS();
}