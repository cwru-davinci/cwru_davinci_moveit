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
#include "hybrid_motion_validator_tester.cpp"

using namespace dual_arm_manipulation_planner_interface;
using namespace hybrid_planner_test;
namespace ob = ompl::base;

TEST(TestHybridRRT, HybridMotionValidator)
{
  ros::NodeHandle node_handle_priv("~");
  std::string objectName = "needle_r";
  robot_model_loader::RobotModelLoader robotModelLoader("robot_description");

  // construct an instance of space information from this state space
  auto se3SS(std::make_shared<ob::SE3StateSpace>());
  auto si(std::make_shared<ob::SpaceInformation>(se3SS));

  HybridMotionValidatorTester tester(si, robotModelLoader.getModel(), objectName);


  int test_num = 0;
  node_handle_priv.getParam("test_num", test_num);

  for (std::size_t i = 0; i < test_num; ++i)
  {
    const robot_state::RobotStatePtr pRStateHome(new robot_state::RobotState(tester.getRobotModel()));
    pRStateHome->setToDefaultValues();
    pRStateHome->update();
    std::size_t variable_count = pRStateHome->getVariableCount();
    EXPECT_EQ(variable_count, pRStateHome->getVariableNames().size());

    std::vector<double> rstate_home_position(variable_count);
    for (std::size_t i = 0; i < variable_count; ++i)
    {
      rstate_home_position[i] = pRStateHome->getVariablePosition(pRStateHome->getVariableNames()[i]);
    }

    tester.publishRobotState(*pRStateHome);

    const robot_state::RobotStatePtr& pRdmRState = tester.sampleRobotState();
    EXPECT_EQ(variable_count, pRdmRState->getVariableNames().size());
    std::vector<double> rstate_random_position(variable_count);
    for (std::size_t i = 0; i < variable_count; ++i)
    {
      rstate_random_position[i] = pRdmRState->getVariablePosition(pRdmRState->getVariableNames()[i]);
    }

    for (std::size_t i = 0; i < variable_count; ++i)
    {
      EXPECT_EQ(pRdmRState->getVariableNames()[i], pRStateHome->getVariableNames()[i]);
    }

    tester.publishRobotState(*pRdmRState);

    bool testResult = tester.testComputeCartesianPath(*pRStateHome, *pRdmRState);
    EXPECT_TRUE(testResult);

    EXPECT_EQ(variable_count, pRStateHome->getVariableNames().size());
    for (std::size_t i = 0; i < variable_count; ++i)
    {
      EXPECT_EQ(rstate_home_position[i], pRStateHome->getVariablePosition(pRStateHome->getVariableNames()[i]));
    }

    EXPECT_EQ(variable_count, pRdmRState->getVariableNames().size());
    for (std::size_t i = 0; i < variable_count; ++i)
    {
      EXPECT_EQ(rstate_random_position[i], pRdmRState->getVariablePosition(pRdmRState->getVariableNames()[i]));
    }
  }

  ROS_INFO("Success Rate: %f", (double) tester.getSucceededNum() / test_num);
  bool success_count = (tester.getSucceededNum() >= 0.9999 * test_num) ? true : false;
  EXPECT_TRUE(success_count);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_hybrid_motion_validator");
  ros::NodeHandle nh;
  ros::Duration(3.0).sleep();
  return RUN_ALL_TESTS();
}
