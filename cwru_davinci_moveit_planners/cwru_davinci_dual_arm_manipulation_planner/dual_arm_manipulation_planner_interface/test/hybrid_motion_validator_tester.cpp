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

namespace dual_arm_manipulation_planner_interface
{
class HybridMotionValidatorTester : public HybridMotionValidator
{
public:
  HybridMotionValidatorTester(const ros::NodeHandle &node_handle,
                              const ros::NodeHandle &node_priv,
                              const std::string &robot_name,
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

private:
  bool samePose(const Eigen::Affine3d &a, const Eigen::Affine3d &b);

//  double eps = std::numeric_limits<double>::epsilon();
  double eps = 1e-5;

  std::string planning_group_ = "psm_one";
};

HybridMotionValidatorTester::HybridMotionValidatorTester(const ros::NodeHandle &node_handle,
                                                         const ros::NodeHandle &node_priv,
                                                         const std::string &robot_name,
                                                         const std::string &object_name,
                                                         const ompl::base::SpaceInformationPtr &si) :
  HybridMotionValidator(node_handle, node_priv, robot_name, object_name, si)
{

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
    if((path_percent - 1.0) < eps)
    {
      cp_start_state->update();

      const Eigen::Affine3d current_tip_pose = cp_start_state->getGlobalLinkTransform(tip_link);
      bool same_mat = current_tip_pose.isApprox(goal_tool_tip_pose, eps);
      bool same_tq = samePose(current_tip_pose, goal_tool_tip_pose);
      if (same_mat || same_tq)
        first = true;
    }
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
    if((path_percent - 1.0) < eps)
    {
      cp_start_state->update();

      const Eigen::Affine3d current_tip_pose = cp_start_state->getGlobalLinkTransform(tip_link);
      bool same_mat = current_tip_pose.isApprox(goal_tool_tip_pose, eps);
      bool same_tq = samePose(current_tip_pose, goal_tool_tip_pose);
      if (same_mat || same_tq)
        second = true;
    }
  }

  if (first && second)
    return true;

  return false;
}

const robot_state::RobotStatePtr& HybridMotionValidatorTester::sampleRobotState()
{
  static robot_state::RobotStatePtr pRState(new robot_state::RobotState(kmodel_));
  const robot_state::JointModelGroup* selected_joint_model_group = pRState->getJointModelGroup(planning_group_);
  pRState->setToRandomPositions(selected_joint_model_group);
  pRState->update();
  return pRState;
}

bool HybridMotionValidatorTester::samePose(const Eigen::Affine3d &a, const Eigen::Affine3d &b)
{
  if (!a.translation().isApprox(b.translation(), eps))
    return false;

  Eigen::Quaterniond qa(a.linear());
  Eigen::Quaterniond qb(b.linear());
  if (!qa.isApprox(qb, eps))
    return false;

  return true;
}

}
