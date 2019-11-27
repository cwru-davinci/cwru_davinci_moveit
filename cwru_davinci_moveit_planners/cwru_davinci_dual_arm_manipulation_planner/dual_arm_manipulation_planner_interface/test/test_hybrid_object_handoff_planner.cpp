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
#include "../../../../../../devel/include/ompl/base/PlannerStatus.h"

#include <dual_arm_manipulation_planner_interface/davinci_needle_handoff_execution_manager.h>

using namespace dual_arm_manipulation_planner_interface;
using namespace hybrid_planner_test;
namespace ob = ompl::base;

class DavinciNeedleHandoffExecutionManagerTester : public DavinciNeedleHandoffExecutionManager
{
public:
  DavinciNeedleHandoffExecutionManagerTester
  (
  const std::string& objectName
  )
  : DavinciNeedleHandoffExecutionManager()
  {
    m_ObjectName = objectName;
    m_PlanningStatus = ob::PlannerStatus::EXACT_SOLUTION;
  }

  bool testExecuteNeedleHandoffTraj
  (
  const PathJointTrajectory& handoffPathJntTraj
  )
  {
    m_HandoffJntTraj = handoffPathJntTraj;
    return executeNeedleHandoffTraj();
  }
};

TEST(TestHybridRRT, HybridObjectHandoffPlanner)
{
  ros::NodeHandle nodeHandle;
  ros::NodeHandle nodeHandlePriv("~");
  robot_model_loader::RobotModelLoader robotModelLoader("robot_description");

  EXPECT_TRUE(nodeHandlePriv.hasParam("object_name"));
  std::string objectName;
  nodeHandlePriv.getParam("object_name", objectName);

  EXPECT_TRUE(nodeHandlePriv.hasParam("initial_support_arm"));
  std::string initialSupportArm;
  nodeHandlePriv.getParam("initial_support_arm", initialSupportArm);

  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr pSimpleGrasp =
  boost::make_shared<cwru_davinci_grasp::DavinciSimpleNeedleGrasper>(nodeHandle,
                                                                     nodeHandlePriv,
                                                                     initialSupportArm,
                                                                     objectName);

  EXPECT_TRUE(pSimpleGrasp->pickNeedle(cwru_davinci_grasp::NeedlePickMode::FINDGOOD, objectName));

  HybridObjectHandoffPlannerTester tester(pSimpleGrasp->getAllPossibleNeedleGrasps(false),
                                          robotModelLoader.getModel());

  DavinciNeedleHandoffExecutionManagerTester managerTester(objectName);
  // tester.testConnectStates();
  PathJointTrajectory handoffPathJntTraj;
  tester.testGetSolutionPathJointTrajectory(handoffPathJntTraj);
  EXPECT_TRUE(managerTester.testExecuteNeedleHandoffTraj(handoffPathJntTraj));

//  const ompl::base::SpaceInformationPtr& si = tester.getSpaceInformation();
//  // create a random start state
//  ob::ScopedState<HybridObjectStateSpace> start(si);
//  ob::ScopedState<HybridObjectStateSpace> goal(si);
//
//  ob::StateSamplerPtr pStateSampler = si->allocStateSampler();
//
//  EXPECT_TRUE(!tester.testSetupProblemDefinition(nullptr, nullptr));
//  bool validSS = false;
//  while (!validSS)
//  {
//    pStateSampler->sampleUniform(start.get());
//    validSS = si->isValid(start.get());
//  }
//
//  bool validGS = false;
//  while (!validGS)
//  {
//    pStateSampler->sampleUniform(goal.get());
//    validGS = si->isValid(goal.get());
//  }
//
//  EXPECT_TRUE(tester.testSetupProblemDefinition(start.get(), goal.get()));
//  EXPECT_TRUE(tester.testSetupPlanner());
//
//  ob::PlannerStatus::StatusType status = tester.testSolve(0.0);
//  EXPECT_EQ(ob::PlannerStatus::StatusType::TIMEOUT, status);
//
//  status = tester.testSolve(100.0);
//  EXPECT_EQ(ob::PlannerStatus::StatusType::EXACT_SOLUTION, status);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_hybrid_object_handoff_planner");
  ros::AsyncSpinner spinner(1);
  ros::Duration(3.0).sleep();
  spinner.start();
  return RUN_ALL_TESTS();
}
