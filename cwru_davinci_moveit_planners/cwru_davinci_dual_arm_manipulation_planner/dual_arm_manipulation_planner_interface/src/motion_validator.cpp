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

#include <cwru_davinci_dual_arm_manipulation_planner/motion_validator.h>
namespace dual_arm_manipulation_planner_interface
{

MotionValidator::MotionValidator(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &node_priv,
                                 const std::string &robot_name,
                                 const std::string &object_name,
                                 const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
                                 const ompl::base::SpaceInformationPtr &si) :
  ompl::base::MotionValidator(si), stateValidityChecker_(node_handle, node_priv, robot_name, object_name, possible_grasps, si)
{

}

bool 	MotionValidator::checkMotion (const State *s1, const State *s2) const
{
  const auto *hs1 = static_cast<const HybridObjectStateSpace::StateType *>(state1);
  const auto *hs2 = static_cast<const HybridObjectStateSpace::StateType *>(state2);

  const int s1_arm_index = hs1->armIndex().value;
  const int s2_arm_index = hs2->armIndex().value;

  const int s1_grasp_index = hs1->graspIndex().value;
  const int s2_grasp_index = hs2->graspIndex().value;

  const int s1_part_id = possible_grasps_[s1_grasp_index].part_id;
  const int s2_part_id = possible_grasps_[s2_grasp_index].part_id;

  // this is the gripper tool tip link frame wrt /base_link
  geometry_msgs::PoseStamped tool_tip_pose_temp = possible_grasps_[hs1->graspIndex().value].grasp.grasp_pose;
  Eigen::Affine3d tool_tip_pose_from;
  tf::poseMsgToEigen(tool_tip_pose_temp.pose, tool_tip_pose_from);

  tool_tip_pose_temp  = possible_grasps_[hs2->graspIndex().value].grasp.grasp_pose;
  Eigen::Affine3d tool_tip_pose_to;
  tf::poseMsgToEigen(tool_tip_pose_temp.pose, tool_tip_pose_to);

  switch(si_->getStateSpacePtr->checkStateDiff(s1, s2))
  {
    case StateDiff::AllSame:
      //
      break;
    case StateDiff::ArmDiff:
      si_->getStateSpacePtr->interpolate(s1, s2, t, cstate);
      break;
    case StateDiff::GraspDiff:
      si_->getStateSpacePtr->interpolate(s1, s2, t, cstate);
      break;
    case StateDiff::PoseDiff:
      break;
    case StateDiff::ArmGraspDiff:
      si_->getStateSpacePtr->interpolate(s1, s2, t, cstate);
      break;
    case StateDiff::ArmPoseDiff:
      si_->getStateSpacePtr->interpolate(s1, s2, t, cstate);
      break;
    case StateDiff::GraspPoseDiff:
      si_->getStateSpacePtr->interpolate(s1, s2, t, cstate);
      break;
    case StateDiff::AllDiff:
      si_->getStateSpacePtr->interpolate(s1, s2, t, cstate);
      break;
  }

  const robot_state::JointModelGroup* joint_model_group = kmodel_->getJointModelGroup(group_name_);

}
}  // namespace