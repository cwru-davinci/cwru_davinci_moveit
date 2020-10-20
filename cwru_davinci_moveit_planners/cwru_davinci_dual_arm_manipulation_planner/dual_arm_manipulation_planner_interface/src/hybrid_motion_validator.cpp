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

#include <dual_arm_manipulation_planner_interface/hybrid_motion_validator.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

using namespace dual_arm_manipulation_planner_interface;

HybridMotionValidator::HybridMotionValidator
(
const ompl::base::SpaceInformationPtr& si,
const robot_model::RobotModelConstPtr& pRobotModel,
const std::string& objectName
)
: ompl::base::MotionValidator(si),
  HybridStateValidityChecker(si,
                             pRobotModel,
                             objectName)
{
  hyStateSpace_->object_transit_planning_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->check_motion_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->hand_off_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->ik_solving_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->collision_checking_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->object_transit_motion_planner_num = 0;

  hyStateSpace_->check_motion_num = 0;

  hyStateSpace_->hand_off_planning_num = 0;

  hyStateSpace_->hand_off_failed_num = 0;
}

bool HybridMotionValidator::checkMotion
(
const ompl::base::State* s1,
const ompl::base::State* s2
) const
{
  hyStateSpace_->check_motion_num += 1;
  auto start = std::chrono::high_resolution_clock::now();

  bool result = false;
  if (!ompl::base::MotionValidator::si_->isValid(s2))
  {
    invalid_++;
  }
  else
  {
    const HybridObjectStateSpace::StateType* pHyState1 = dynamic_cast<const HybridObjectStateSpace::StateType*>(s1);
    const HybridObjectStateSpace::StateType* pHyState2 = dynamic_cast<const HybridObjectStateSpace::StateType*>(s2);

    if (!pHyState1 || !pHyState2)
    {
      printf("HybridMotionValidator: Invalid states to be connected \n");
      return false;
    }

    const robot_state::RobotStatePtr start_state = std::make_shared<robot_state::RobotState>(kmodel_);
    if (!start_state || !hybridStateToRobotState(pHyState1, start_state))
    {
      return result;
    }
    const robot_state::RobotStatePtr goal_state = std::make_shared<robot_state::RobotState>(kmodel_);
    if (!goal_state || !hybridStateToRobotState(pHyState2, goal_state))
    {
      return result;
    }

    if (!start_state->hasAttachedBody(m_ObjectName) || !goal_state->hasAttachedBody(m_ObjectName))
    {
      start_state->clearAttachedBodies();
      goal_state->clearAttachedBodies();
      return result;
    }

    // active group is the arm supporting the needle
    const std::string supportGroupS1 = (pHyState1->armIndex().value == 1) ? "psm_one" : "psm_two";
    const std::string supportGroupS2 = (pHyState2->armIndex().value == 1) ? "psm_one" : "psm_two";

    switch (hyStateSpace_->checkStateDiff(pHyState1, pHyState2))
    {
      case StateDiff::AllSame:
        result = true;
        break;
      case StateDiff::PoseDiffArmAndGraspSame:
      {
        result = planObjectTransit(*start_state, *goal_state, supportGroupS1);
        if (!result)
          ROS_WARN("HybridMotionValidator: Local planning failed at the stage of ObjectTransit");
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        hyStateSpace_->object_transit_planning_duration_ += elapsed;
        break;
      }
      case StateDiff::ArmAndGraspDiffPoseSame:
        result = planHandoff(*start_state, *goal_state, supportGroupS1, supportGroupS2);
        if (!result)
          hyStateSpace_->hand_off_failed_num += 1;
        break;
      default:
        // should not be there
        break;
    }

    result ? valid_++ : invalid_++;
    start_state->clearAttachedBodies();
    goal_state->clearAttachedBodies();
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  hyStateSpace_->check_motion_duration_ += elapsed;

  return result;
}

bool HybridMotionValidator::planHandoff
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state,
const std::string& ss_active_group,
const std::string& gs_active_group
) const
{
  hyStateSpace_->hand_off_planning_num += 1;
  auto start = std::chrono::high_resolution_clock::now();

  bool able_to_grasp = false;
  bool able_to_release = false;
  bool able_to_handoff = false;

  // an intermediate state when needle is supporting by two grippers
  const robot_state::RobotStatePtr handoff_state = std::make_shared<robot_state::RobotState>(start_state);

  std::vector<double> gs_jt_position;
  goal_state.copyJointGroupPositions(gs_active_group, gs_jt_position);
  handoff_state->setJointGroupPositions(gs_active_group, gs_jt_position);
  setMimicJointPositions(handoff_state, gs_active_group);

  const moveit::core::AttachedBody* ss_needle_body = start_state.getAttachedBody(m_ObjectName);
  const moveit::core::AttachedBody* gs_needle_body = goal_state.getAttachedBody(m_ObjectName);

  std::set<std::string> touch_links = ss_needle_body->getTouchLinks();
  touch_links.insert(gs_needle_body->getTouchLinks().begin(), gs_needle_body->getTouchLinks().end());

  trajectory_msgs::JointTrajectory dettach_posture = ss_needle_body->getDetachPosture();
  trajectory_msgs::JointTrajectory gs_dettach_posture = gs_needle_body->getDetachPosture();

  dettach_posture.joint_names.insert(dettach_posture.joint_names.end(),
                                     gs_dettach_posture.joint_names.begin(),
                                     gs_dettach_posture.joint_names.end());

  dettach_posture.points.insert(dettach_posture.points.end(),
                                gs_dettach_posture.points.begin(),
                                gs_dettach_posture.points.end());


  handoff_state->attachBody(gs_needle_body->getName(), gs_needle_body->getShapes(),
                            gs_needle_body->getFixedTransforms(), touch_links,
                            gs_needle_body->getAttachedLinkName(), gs_needle_body->getDetachPosture());
  handoff_state->update();

  if (!noCollision(*handoff_state))
  {
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    hyStateSpace_->hand_off_duration_ += elapsed;
    return able_to_handoff;
  }

  able_to_grasp = planNeedleGrasping(start_state, *handoff_state, gs_active_group);
  if (able_to_grasp)
  {
    able_to_release = planNeedleReleasing(*handoff_state, goal_state, ss_active_group);
    if (able_to_release)
      able_to_handoff = true;
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  hyStateSpace_->hand_off_duration_ += elapsed;
  return able_to_handoff;
}

bool HybridMotionValidator::planNeedleGrasping
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& handoff_state,
const std::string& gs_active_group
) const
{
  // planning in a back order fashion
  robot_state::RobotStatePtr pre_grasp_state = std::make_shared<robot_state::RobotState>(start_state);
  if (!planPreGraspStateToGraspedState(pre_grasp_state, handoff_state, gs_active_group))
  {
    ROS_WARN("HybridMotionValidator: Local planning failed at the stage of PreGraspState To GraspedState");
    return false;
  }
  if (!planSafeStateToPreGraspState(start_state, *pre_grasp_state, gs_active_group))
  {
    ROS_WARN("HybridMotionValidator: Local planning failed at the stage of SafeState To PreGraspState");
    return false;
  }
  return true;
}

bool HybridMotionValidator::planNeedleReleasing
(
const robot_state::RobotState& handoff_state,
const robot_state::RobotState& goal_state,
const std::string& ss_active_group
) const
{
  robot_state::RobotStatePtr ungrasped_state = std::make_shared<robot_state::RobotState>(goal_state);
  if (!planGraspStateToUngraspedState(handoff_state, ungrasped_state, ss_active_group))
  {
    ROS_WARN("HybridMotionValidator: Local planning failed at the stage of GraspState To UngraspedState");
    return false;
  }
  if (!planUngraspedStateToSafeState(*ungrasped_state, goal_state, ss_active_group))
  {
    ROS_WARN("HybridMotionValidator: Local planning failed at the stage of UngraspedState To SafeState");
    return false;
  }
  return true;
}

bool HybridMotionValidator::planPreGraspStateToGraspedState
(
robot_state::RobotStatePtr& pre_grasp_state,
const robot_state::RobotState& handoff_state,
const std::string& planning_group
) const
{
  // make a pre_grasp_state
  const robot_state::JointModelGroup* arm_joint_group = pre_grasp_state->getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d pregrasp_tool_tip_pose = grasped_tool_tip_pose;
  Eigen::Vector3d unit_approach_dir(0.0, 0.0, 1.0);  // grasp approach along the +z-axis of tip frame

  auto start_ik = std::chrono::high_resolution_clock::now();

  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridMotionValidator::isRobotStateValid,
                                                                                  this,
                                                                                  boost::cref(*planning_scene_),
                                                                                  boost::cref(planning_group),
                                                                                  true, true, _1, _2, _3);
  bool found_ik = false;
  double distance = 0.01;
  while (distance <= 0.015)
  {
    Eigen::Vector3d approach_dir = grasped_tool_tip_pose.linear() * (distance * unit_approach_dir);
    pregrasp_tool_tip_pose.translation() = grasped_tool_tip_pose.translation() - approach_dir;
    std::size_t attempts = 1;
    double timeout = 0.005;
    found_ik = pre_grasp_state->setFromIK(arm_joint_group, pregrasp_tool_tip_pose, attempts, timeout, stateValidityCallbackFn);
    if (found_ik)
      break;
    distance += 0.001;
  }
  if (!found_ik)
  {
    auto finish_ik = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_ik - start_ik;
    hyStateSpace_->ik_solving_duration_ += elapsed;
    return found_ik;
  }

  const std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  pre_grasp_state->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (std::size_t i = 0; i < eef_joint_position.size(); ++i)
  {
    eef_joint_position[i] = 0.5;
  }
  pre_grasp_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  pre_grasp_state->update();

  std::vector<robot_state::RobotStatePtr> traj;
  double translation_step_max = 0.001, rotation_step_max = 0.0;
  moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
  moveit::core::JumpThreshold jump_threshold;

  double found_cartesian_path = pre_grasp_state->computeCartesianPath(pre_grasp_state->getJointModelGroup(planning_group),
                                                                      traj,
                                                                      tip_link,
                                                                      grasped_tool_tip_pose,
                                                                      true,
                                                                      max_step,
                                                                      jump_threshold,
                                                                      stateValidityCallbackFn);

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (!(fabs(found_cartesian_path - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return clear_path;
  }

  pre_grasp_state = std::move(traj[0]);
  for (std::size_t i = 0; i < eef_joint_position.size(); ++i)
  {
    eef_joint_position[i] = 0.0;
  }
  pre_grasp_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  pre_grasp_state->update();
  clear_path = true;
  return clear_path;
}


bool HybridMotionValidator::planSafeStateToPreGraspState
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& pre_grasp_state,
const std::string& planning_group
) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(pre_grasp_state);
  const robot_state::JointModelGroup* arm_joint_group = pre_grasp_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_home = start_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;
  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridStateValidityChecker::isRobotStateValid,
                                                                                  this,
                                                                                  boost::cref(*planning_scene_),
                                                                                  boost::cref(planning_group),
                                                                                  true, true, _1, _2, _3);

  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     tool_tip_home,
                                                                     true,
                                                                     0.001,
                                                                     0.0,
                                                                     stateValidityCallbackFn);

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (!((found_cartesian_path - 0.8) >= std::numeric_limits<double>::epsilon()))
  {
    return clear_path;
  }

  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planGraspStateToUngraspedState
(
const robot_state::RobotState& handoff_state,
robot_state::RobotStatePtr& ungrasped_state,
const std::string& planning_group
) const
{
  // make a ungrasped_state
  const robot_state::JointModelGroup* arm_joint_group = handoff_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d ungrasped_tool_tip_pose = grasped_tool_tip_pose;
  Eigen::Vector3d retreat_dir(0.0, 0.0, -1.0);  // ungrasp retreat along the -z-axis of tip frame
  double distance = 0.01;
  retreat_dir = grasped_tool_tip_pose.linear() * (distance * retreat_dir);
  ungrasped_tool_tip_pose.translation() += retreat_dir;

  auto start_ik = std::chrono::high_resolution_clock::now();

  std::vector<double> fromSupportGroupJntPosition;
  handoff_state.copyJointGroupPositions(planning_group, fromSupportGroupJntPosition);  // this will not copy mimic joints' value
  ungrasped_state->setJointGroupPositions(planning_group, fromSupportGroupJntPosition);

  const std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  ungrasped_state->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (std::size_t i = 0; i < eef_joint_position.size(); ++i)
  {
    eef_joint_position[i] = 0.5;
  }
  ungrasped_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  setMimicJointPositions(ungrasped_state, planning_group);  // this line cannot be removed, as copyJointGroupPosition does not copy mimic joints's value
  ungrasped_state->update();

  if (!noCollision(*ungrasped_state, planning_group, false, true))
    return false;
  std::vector<robot_state::RobotStatePtr> traj;
  double translation_step_max = 0.001, rotation_step_max = 0.0;
  moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
  moveit::core::JumpThreshold jump_threshold;

  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridStateValidityChecker::isRobotStateValid,
                                                                                  this,
                                                                                  boost::cref(*planning_scene_),
                                                                                  boost::cref(planning_group),
                                                                                  false, true, _1, _2, _3);
  double found_cartesian_path = ungrasped_state->computeCartesianPath(ungrasped_state->getJointModelGroup(planning_group),
                                                                      traj,
                                                                      tip_link,
                                                                      ungrasped_tool_tip_pose,
                                                                      true,
                                                                      max_step,
                                                                      jump_threshold,
                                                                      stateValidityCallbackFn);

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;

  ungrasped_state = std::move(traj.back());
  if ((found_cartesian_path - 0.4) >= std::numeric_limits<double>::epsilon())
    ungrasped_state->setToDefaultValues(ungrasped_state->getJointModelGroup(eef_group_name), eef_group_name + "_home");

  ungrasped_state->update();
  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planUngraspedStateToSafeState
(
const robot_state::RobotState& ungrasped_state,
const robot_state::RobotState& goal_state,
const std::string& planning_group
) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(ungrasped_state);
  const robot_state::JointModelGroup* arm_joint_group = goal_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;
  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridStateValidityChecker::isRobotStateValid,
                                                                                  this,
                                                                                  boost::cref(*planning_scene_),
                                                                                  boost::cref(planning_group),
                                                                                  true, true, _1, _2, _3);

  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     tool_tip_pose,
                                                                     true,
                                                                     0.001,
                                                                     0.0,
                                                                     stateValidityCallbackFn);

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (!((found_cartesian_path - 0.8) >= std::numeric_limits<double>::epsilon()))
  {
    return clear_path;
  }

  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planObjectTransit
(
const robot_state::RobotState& start_state,
const robot_state::RobotState& goal_state,
const std::string& planning_group
) const
{
  hyStateSpace_->object_transit_motion_planner_num += 1;
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state = std::make_shared<robot_state::RobotState>(start_state);
  const robot_state::JointModelGroup* arm_joint_group = goal_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel* tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;
  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridStateValidityChecker::isRobotStateValid,
                                                                                  this,
                                                                                  boost::cref(*planning_scene_),
                                                                                  boost::cref(planning_group),
                                                                                  true, true, _1, _2, _3);

  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     tool_tip_pose,
                                                                     true,
                                                                     0.001,
                                                                     0.0,
                                                                     stateValidityCallbackFn);
  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (!(fabs(found_cartesian_path - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return clear_path;
  }

  clear_path = true;
  return clear_path;
}
