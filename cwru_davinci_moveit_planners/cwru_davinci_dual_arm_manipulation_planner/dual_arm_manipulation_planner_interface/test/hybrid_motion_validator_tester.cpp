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
#include <moveit/dual_arm_manipulation_planner_interface//hybrid_motion_validator.h>

class HybridMotionValidatorTester : public HybridMotionValidator
{
public:
  HybridMotionValidator(const ros::NodeHandle &node_handle,
                        const ros::NodeHandle &node_priv,
                        const std::string &robot_name,
                        const std::string &object_name,
                        const ompl::base::SpaceInformationPtr &si);

  ~HybridMotionValidatorTester(){}

  void testComputeCartesianPath(const robot_state::RobotState& start_state,
                                const robot_state::RobotState& goal_state);

private:
  bool samePose(const Eigen::Affine3d& a, const Eigen::Affine3d& b);

  typedef std::numeric_limits<double>::epsilon() eps;
};

HybridMotionValidatorTester::HybridMotionValidator(const ros::NodeHandle &node_handle,
                                                   const ros::NodeHandle &node_priv,
                                                   const std::string &robot_name,
                                                   const std::string &object_name,
                                                   const ompl::base::SpaceInformationPtr &si) :
                                                   HybridMotionValidator(node_handle, node_priv, robot_name, object_name, si)
{
}

void HybridMotionValidatorTester::testComputeCartesianPath(const robot_state::RobotState& start_state,
                                                           const robot_state::RobotState& goal_state)
{
  std::string planning_group = "psm_one"
  {
    robot_state::RobotState cp_start_state = start_state;
    const robot_state::JointModelGroup *arm_joint_group = goal_state.getJointModelGroup(planning_group);
    const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
    const Eigen::Affine3d goal_tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

    std::vector<robot_state::RobotStatePtr> traj;
    double translation_step_max = 0.005, rotation_step_max = 0.005;
    moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
    double jt_revolute = 0.0, jt_prismatic = 0.0, jump_threshold_factor = 0.1;
    moveit::core::JumpThreshold jump_threshold;

    bool found_cartesian_path = cp_start_state.computeCartesianPath(cp_start_state.getJointModelGroup(planning_group),
                                                                    traj,
                                                                    tip_link,
                                                                    goal_tool_tip_pose,
                                                                    true,
                                                                    max_step,
                                                                    jump_threshold);
    cp_start_state.update();

    {
      const Eigen::Affine3d current_tip_pose = cp_start_state.getGlobalLinkTransform(tip_link);
      bool is_same = current_tip_pose.isApprox(goal_tool_tip_pose, 0.0001);
    }

    {

      const Eigen::Affine3d current_tip_pose = cp_start_state.getGlobalLinkTransform(tip_link);
    }

  }

  {
    robot_state::RobotState cp_start_state = start_state;
    std::vector<robot_state::RobotStatePtr> traj;
    double translation_step_max = 0.001, rotation_step_max = 0.0;
    moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
    double jt_revolute = 0.0, jt_prismatic = 0.0, jump_threshold_factor = 0.1;
    moveit::core::JumpThreshold jump_threshold;
    bool found_cartesian_path = pre_grasp_state.computeCartesianPath(arm_joint_group, traj, tip_link,
                                                                     grasped_tool_tip_pose, true, max_step,
                                                                     jump_threshold);
  }
}

bool HybridMotionValidatorTester::samePose(const Eigen::Affine3d& a, const Eigen::Affine3d& b)
{
  if(!a.translation.isApprox(b.translation(), eps))
    return false;

  Eigen::Quaternion qa(a.linear());
  Eigen::Quaternion qb(b.linear());
  if(!qa.isApprox(qb, eps))
    return false;

  return true;
}
