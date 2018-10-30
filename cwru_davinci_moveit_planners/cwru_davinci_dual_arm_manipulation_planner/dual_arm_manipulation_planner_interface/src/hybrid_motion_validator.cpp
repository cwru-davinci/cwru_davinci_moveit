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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Su Lu <sxl924@case.edu>
 * Description: The class to do state motion validity check
 */

#include <moveit/dual_arm_manipulation_planner_interface//hybrid_motion_validator.h>
#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

namespace dual_arm_manipulation_planner_interface
{

HybridMotionValidator::HybridMotionValidator(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &node_priv,
                                 const std::string &robot_name,
                                 const std::string &object_name,
                                 const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
                                 const ompl::base::SpaceInformationPtr &si) :
  ompl::base::MotionValidator(si), stateValidityChecker_(node_handle, node_priv, robot_name, object_name, possible_grasps, si)
{

}

bool 	HybridMotionValidator::checkMotion (const ompl::base::State *s1, const ompl::base::State *s2) const
{
  if (!si_->isValid(s2))
  {
    invalid_++;
    return false;
  }

  bool result = true;

  const auto *hs1 = static_cast<const HybridObjectStateSpace::StateType *>(s1);
  const auto *hs2 = static_cast<const HybridObjectStateSpace::StateType *>(s2);

  // this is the gripper tool tip link frame wrt /base_link
  geometry_msgs::PoseStamped tool_tip_pose_temp = si_->getStateSpace()->as<HybridObjectStateSpace>()->possible_grasps_[hs1->graspIndex().value].grasp.grasp_pose;
  Eigen::Affine3d tool_tip_pose_from;
  tf::poseMsgToEigen(tool_tip_pose_temp.pose, tool_tip_pose_from);

  tool_tip_pose_temp  = si_->getStateSpace()->as<HybridObjectStateSpace>()->possible_grasps_[hs2->graspIndex().value].grasp.grasp_pose;
  Eigen::Affine3d tool_tip_pose_to;
  tf::poseMsgToEigen(tool_tip_pose_temp.pose, tool_tip_pose_to);

//  si_->getStateSpace()->as<HybridObjectStateSpace>()->interpolate(s1, s2, t, cstate);
  std::string group_s1 = (hs1->graspIndex().value == 1) ?  "psm_one" : "psm_two";
  std::string group_s2 = (hs2->graspIndex().value == 1) ?  "psm_one" : "psm_two";

  const robot_state::JointModelGroup* joint_model_group_s1 = kmodel_->getJointModelGroup(group_s1);
  const robot_state::JointModelGroup* joint_model_group_s2 = kmodel_->getJointModelGroup(group_s2);

  switch(si_->getStateSpace()->as<HybridObjectStateSpace>()->checkStateDiff(hs1, hs2))
  {
    case StateDiff::AllSame:
      break;
    case StateDiff::ArmDiffGraspAndPoseSame:
      // Todo handoff operation check
      break;
    case StateDiff::GraspDiffArmAndPoseSame:
      // Todo handoff operation check
      break;
    case StateDiff::PoseDiffArmAndGraspSame:

      planning_interface::MotionPlanRequest req;
      planning_interface::MotionPlanResponse res;
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "panda_link0";
      pose.pose.position.x = 0.3;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = 0.75;
      pose.pose.orientation.w = 1.0;

      break;
    case StateDiff::ArmAndGraspDiffPoseSame:
      // Todo handoff operation check
      break;
    case StateDiff::ArmAndPoseDiffGraspSame:
      // Todo handoff operation check
      break;
    case StateDiff::GraspAndPoseDiffArmSame:
      // Todo handoff operation check
      break;
    case StateDiff::AllDiff:
      // Todo handoff operation check
      break;
  }



  if (result)
  {
    valid_++;
  }
  else
  {
    invalid_++;
  }

  return result;
}

bool ompl::base::DiscreteMotionValidator::checkMotion(const State *s1, const State *s2) const
{
  /* assume motion starts in a valid configuration so s1 is valid */
  if (!si_->isValid(s2))
  {
    invalid_++;
    return false;
  }

  bool result = true;
  int nd = stateSpace_->validSegmentCount(s1, s2);

  /* initialize the queue of test positions */
  std::queue<std::pair<int, int>> pos;
  if (nd >= 2)
  {
    pos.push(std::make_pair(1, nd - 1));

    /* temporary storage for the checked state */
    State *test = si_->allocState();

    /* repeatedly subdivide the path segment in the middle (and check the middle) */
    while (!pos.empty())
    {
      std::pair<int, int> x = pos.front();

      int mid = (x.first + x.second) / 2;
      stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);

      if (!si_->isValid(test))
      {
        result = false;
        break;
      }

      pos.pop();

      if (x.first < mid)
        pos.push(std::make_pair(x.first, mid - 1));
      if (x.second > mid)
        pos.push(std::make_pair(mid + 1, x.second));
    }

    si_->freeState(test);
  }

  if (result)
    valid_++;
  else
    invalid_++;

  return result;
}

}  // namespace