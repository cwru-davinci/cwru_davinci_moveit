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
#include <moveit/dual_arm_manipulation_planner_interface/hybrid_motion_validator.h>
#include <gtest/gtest.h>

using namespace dual_arm_manipulation_planner_interface;

namespace hybrid_planner_test
{
class HybridMotionValidatorTester : public HybridMotionValidator
{
public:
  HybridMotionValidatorTester(const std::string &robot_name,
                              const std::string &object_name,
                              const ompl::base::SpaceInformationPtr &si);

  ~HybridMotionValidatorTester()
  {}

  const robot_state::RobotStatePtr& sampleRobotState();

  bool testComputeCartesianPath(const robot_state::RobotState &start_state,
                                const robot_state::RobotState &goal_state);

  inline const robot_model::RobotModelConstPtr getRobotModel() const
  {
    return kmodel_;
  }

  inline int getSucceededNum()
  {
    return succeeded_num_;
  }

  using HybridMotionValidator::publishRobotState;

private:
  double eps = 1e-5;

  std::string planning_group_ = "psm_one";

  int succeeded_num_ = 0;
};

HybridMotionValidatorTester::HybridMotionValidatorTester(const std::string &robot_name,
                                                         const std::string &object_name,
                                                         const ompl::base::SpaceInformationPtr &si) :
  HybridMotionValidator(robot_name, object_name, si)
{
// left blank
}

bool HybridMotionValidatorTester::testComputeCartesianPath(const robot_state::RobotState &start_state,
                                                           const robot_state::RobotState &goal_state)
{
  bool first = false;
  {
    const robot_state::RobotStatePtr cp_start_state(new robot_state::RobotState(start_state));
    const robot_state::JointModelGroup *arm_joint_group = goal_state.getJointModelGroup(planning_group_);
    const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
    const Eigen::Affine3d goal_tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

    std::vector <robot_state::RobotStatePtr> traj;
    double translation_step_max = 0.001, rotation_step_max = 0.001;
    moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
//    double jt_revolute = 0.0, jt_prismatic = 0.0, jump_threshold_factor = 0.1;
    moveit::core::JumpThreshold jump_threshold;

    double path_percent = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group_),
                                                               traj,
                                                               tip_link,
                                                               goal_tool_tip_pose,
                                                               true,
                                                               max_step,
                                                               jump_threshold);
    first = ((path_percent - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 2);
  }

  std::size_t variable_count = start_state.getVariableCount();

  bool same_count_var = (variable_count == start_state.getVariableNames().size()) ? true : false;
  EXPECT_TRUE(same_count_var);

  std::vector<double> rstate_home_position(variable_count);
  for (std::size_t i = 0; i < variable_count; i++)
  {
    rstate_home_position[i] = start_state.getVariablePosition(start_state.getVariableNames()[i]);
  }

  bool second = false;
  {
    const robot_state::RobotStatePtr cp_start_state(new robot_state::RobotState(start_state));
    const robot_state::JointModelGroup *arm_joint_group = goal_state.getJointModelGroup(planning_group_);
    const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
    const Eigen::Affine3d goal_tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

    std::vector <robot_state::RobotStatePtr> traj;
    double path_percent = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group_),
                                                               traj,
                                                               tip_link,
                                                               goal_tool_tip_pose,
                                                               true,
                                                               0.001,
                                                               0.0);
    second = ((path_percent - 1.0) < eps) ? true : false;
    EXPECT_TRUE(traj.size() > 2);
  }

  same_count_var = (variable_count == start_state.getVariableNames().size()) ? true : false;
  EXPECT_TRUE(same_count_var);

  for (std::size_t i = 0; i < variable_count; i++)
  {
    EXPECT_EQ(rstate_home_position[i], start_state.getVariablePosition(start_state.getVariableNames()[i]));
  }

  if (first && second)
  {
    succeeded_num_ += 1;
    ROS_INFO("Times of succeeded %d", succeeded_num_);
    return true;
  }

  return false;
}

const robot_state::RobotStatePtr& HybridMotionValidatorTester::sampleRobotState()
{
  static robot_state::RobotStatePtr pRState(new robot_state::RobotState(kmodel_));
  pRState->setToDefaultValues();
  const robot_state::JointModelGroup* selected_joint_model_group = pRState->getJointModelGroup(planning_group_);
  pRState->setToRandomPositions(selected_joint_model_group);
  pRState->update();
  return pRState;
}

}
