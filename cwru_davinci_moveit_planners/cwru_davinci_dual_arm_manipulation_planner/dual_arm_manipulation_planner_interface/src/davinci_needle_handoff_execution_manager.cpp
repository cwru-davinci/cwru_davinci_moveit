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
 * Description: The class to execute the joint trajectory of needle handoff on daVinci surgical robot
 */

#include <dual_arm_manipulation_planner_interface/davinci_needle_handoff_execution_manager.h>
#include <dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

using namespace dual_arm_manipulation_planner_interface;

DavinciNeedleHandoffExecution::DavinciNeedleHandoffExecution(const ompl::geometric::PathGeometric &sln_path,
                                                             int joint_space_dim,
                                                             const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
                                                             const cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr &needleGrasper)
  : sln_path_(sln_path), joint_space_dim_(joint_space_dim), grasp_poses_(grasp_poses), needleGrasper_(needleGrasper)
{
  readData();
}

bool DavinciNeedleHandoffExecution::executeNeedleHandoffTrajy()
{
  if (handoff_traj_sequence_.empty())
  {
    ROS_INFO("No hand off trajectory");
    return false;
  }


  for (std::size_t i = 0; i < handoff_traj_sequence_.size()-1; i++)
  {
    const std::string ss_active_group = handoff_traj_sequence_[i].support_arm;
    const std::string gs_active_group = handoff_traj_sequence_[i+1].support_arm;

    switch (checkTrajDiff(ss_active_group, gs_active_group))
    {
      case TrajDiff::SameArm:
        executeNeedleTransfer(handoff_traj_sequence_[i], handoff_traj_sequence_[i+1]);
        break;
      case TrajDiff::DiffArm:
        executeNeedleTransit(handoff_traj_sequence_[i], handoff_traj_sequence_[i+1]);
        break;
    }
  }

}

bool DavinciNeedleHandoffExecution::readData()
{
  handoff_state_.clear();
  handoff_state_ = sln_path_.getStates();

  int num_state = sln_path_.getStateCount();
  if (handoff_state_.size() != num_state || num_state == 0)
    return false;

  handoff_traj_sequence_.clear();
  for (int i = 0; i < num_state; i++)
  {
    const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(handoff_state_[i]);

    HandoffSlnPath sln_state(joint_space_dim_);
    if(hs->jointVariables().values)
    {
//    sln_state.joint_values = new double[joint_space_dim_];
      for (int j = 0; j < joint_space_dim_; j++)
      {
        sln_state.joint_values[j] = hs->jointVariables().values[j];
      }
      sln_state.support_arm = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";
      sln_state.rest_arm = (hs->armIndex().value == 1) ? "psm_two" : "psm_one";
      sln_state.needle_pose.pose.position.x = hs->se3State().getX();
      sln_state.needle_pose.pose.position.y = hs->se3State().getY();
      sln_state.needle_pose.pose.position.z = hs->se3State().getZ();
      sln_state.needle_pose.pose.orientation.x = hs->se3State().rotation().x;
      sln_state.needle_pose.pose.orientation.y = hs->se3State().rotation().y;
      sln_state.needle_pose.pose.orientation.z = hs->se3State().rotation().z;
      sln_state.needle_pose.pose.orientation.w = hs->se3State().rotation().w;
    }
    else
      sln_state.joint_values = 0;
    handoff_traj_sequence_.push_back(sln_state);
  }
}

bool DavinciNeedleHandoffExecution::executeNeedleTransfer(const HandoffSlnPath &ss_joint_traj, const HandoffSlnPath &gs_joint_traj)
{

  moveit::planning_interface::MoveGroupInterface move_group(gs_joint_traj.support_arm);

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  current_state->setJointGroupPositions(gs_joint_traj.support_arm, gs_joint_traj.joint_values);
  current_state->setToDefaultValues(current_state->getJointModelGroup(gs_joint_traj.rest_arm), gs_joint_traj.rest_arm+"_home");
  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(*current_state);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("NeedleTransferExecution", "Visualizing (joint space goal) %s", success ? "" : "FAILED");
  return success;
}

bool DavinciNeedleHandoffExecution::executeNeedleTransit(const HandoffSlnPath &ss_joint_traj, const HandoffSlnPath &gs_joint_traj)
{
  if(!executeNeedleGrasping(gs_joint_traj))
    return false;
  if(!executeNeedleUngrasping(ss_joint_traj))
    return false;
  return true;
}

TrajDiff DavinciNeedleHandoffExecution::checkTrajDiff(const std::string& support_arm_1, const std::string& support_arm_2)
{
  if(support_arm_1 == support_arm_2)
    return TrajDiff::SameArm;
  return TrajDiff::DiffArm;
}

bool DavinciNeedleHandoffExecution::executeNeedleGrasping(const HandoffSlnPath& joint_traj)
{
  needleGrasper_->changePlanningGroup(joint_traj.support_arm);
  if (!needleGrasper_->pickNeedle(joint_traj.needle_pose, "needle_r", cwru_davinci_grasp::NeedlePickMode::DEFINED,
                                  grasp_poses_[joint_traj.grasp_id], true, false))
    return false;
}

bool DavinciNeedleHandoffExecution::executeNeedleUngrasping(const HandoffSlnPath& joint_traj)
{
  needleGrasper_->changePlanningGroup(joint_traj.support_arm);
  if(!needleGrasper_->releaseNeedle());
    return false;


}
