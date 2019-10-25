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

#include <moveit/dual_arm_manipulation_planner_interface/hybrid_motion_validator.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

// collision
#include <moveit/collision_detection/collision_tools.h>
//#include <interactivity/interactive_robot.h>

using namespace dual_arm_manipulation_planner_interface;

HybridMotionValidator::HybridMotionValidator(const std::string &robot_name,
                                             const std::string &object_name,
                                             const ompl::base::SpaceInformationPtr &si) :
  ompl::base::MotionValidator(si),
  HybridStateValidityChecker(robot_name, object_name, si)
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

bool HybridMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
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
    const auto *hs1 = static_cast<const HybridObjectStateSpace::StateType *>(s1);
    const auto *hs2 = static_cast<const HybridObjectStateSpace::StateType *>(s2);

    // active group is the arm supporting the needle
    const std::string active_group_s1 = (hs1->armIndex().value == 1) ? "psm_one" : "psm_two";
    const std::string rest_group_s1 = (active_group_s1 == "psm_one") ? "psm_two" : "psm_one";

    const robot_state::RobotStatePtr start_state(new robot_state::RobotState(kmodel_));
    if(!start_state)
    {
      return result;
    }

    start_state->setToDefaultValues();
    start_state->setJointGroupPositions(active_group_s1, hs1->jointVariables().values);
    setMimicJointPositions(start_state, active_group_s1);
    start_state->setToDefaultValues(start_state->getJointModelGroup(rest_group_s1), rest_group_s1 + "_home");
    const std::string rest_group_s1_eef_name = start_state->getJointModelGroup(
      rest_group_s1)->getAttachedEndEffectorNames()[0];
    start_state->setToDefaultValues(start_state->getJointModelGroup(rest_group_s1_eef_name),
                                    rest_group_s1_eef_name + "_home");

    const std::string active_group_s2 = (hs2->armIndex().value == 1) ? "psm_one" : "psm_two";
    const std::string rest_group_s2 = (active_group_s2 == "psm_one") ? "psm_two" : "psm_one";

    const robot_state::RobotStatePtr goal_state(new robot_state::RobotState(kmodel_));
    if(!goal_state)
    {
      return result;
    }

    goal_state->setToDefaultValues();
    goal_state->setJointGroupPositions(active_group_s2, hs2->jointVariables().values);
    setMimicJointPositions(goal_state, active_group_s2);
    goal_state->setToDefaultValues(goal_state->getJointModelGroup(rest_group_s2), rest_group_s2 + "_home");
    std::string rest_group_s2_eef_name = goal_state->getJointModelGroup(
      rest_group_s2)->getAttachedEndEffectorNames()[0];
    goal_state->setToDefaultValues(goal_state->getJointModelGroup(rest_group_s2_eef_name),
                                   rest_group_s2_eef_name + "_home");

    moveit::core::AttachedBody *s1_needle = createAttachedBody(active_group_s1,
                                                               object_name_,
                                                               hs1->graspIndex().value);
    moveit::core::AttachedBody *s2_needle = createAttachedBody(active_group_s2,
                                                               object_name_,
                                                               hs2->graspIndex().value);
    if(!s1_needle || !s2_needle)
    {
      return result;
    }
    start_state->attachBody(s1_needle);
    start_state->update();
    goal_state->attachBody(s2_needle);
    goal_state->update();

    // publishRobotState(*start_state);
    // publishRobotState(*goal_state);

    if (!start_state->hasAttachedBody(object_name_) || !goal_state->hasAttachedBody(object_name_))
    {
      start_state->clearAttachedBodies();
      goal_state->clearAttachedBodies();
      return result;
    }

    switch (hyStateSpace_->checkStateDiff(hs1, hs2))
    {
      case StateDiff::AllSame:
        result = true;
        break;
      case StateDiff::PoseDiffArmAndGraspSame:
      {
        result = planObjectTransit(*start_state, *goal_state, active_group_s1);
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        hyStateSpace_->object_transit_planning_duration_ += elapsed;
        break;
      }
      case StateDiff::ArmAndGraspDiffPoseSame:
        result = planHandoff(*start_state, *goal_state, active_group_s1, active_group_s2);
        if(!result)
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

bool HybridMotionValidator::planHandoff(const robot_state::RobotState &start_state,
                                        const robot_state::RobotState &goal_state,
                                        const std::string &ss_active_group,
                                        const std::string &gs_active_group) const
{
  hyStateSpace_->hand_off_planning_num += 1;
  auto start = std::chrono::high_resolution_clock::now();

  bool able_to_grasp = false;
  bool able_to_release = false;
  bool able_to_handoff = false;

  // an intermediate state when needle is supporting by two grippers
  const robot_state::RobotStatePtr handoff_state(new robot_state::RobotState(start_state));

  std::vector<double> gs_jt_position;
  goal_state.copyJointGroupPositions(gs_active_group, gs_jt_position);
  handoff_state->setJointGroupPositions(gs_active_group, gs_jt_position);
  setMimicJointPositions(handoff_state, gs_active_group);

  const moveit::core::AttachedBody *ss_needle_body = start_state.getAttachedBody(object_name_);
  const moveit::core::AttachedBody *gs_needle_body = goal_state.getAttachedBody(object_name_);

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
    // publishRobotState(*handoff_state);
    return able_to_handoff;
  }

  // publishRobotState(*handoff_state);

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

bool HybridMotionValidator::planNeedleGrasping(const robot_state::RobotState &start_state,
                                               const robot_state::RobotState &handoff_state,
                                               const std::string &gs_active_group) const
{
  // planning in a back order fashion
  robot_state::RobotStatePtr pre_grasp_state(new robot_state::RobotState(start_state));
  if (!planPreGraspStateToGraspedState(pre_grasp_state, handoff_state, gs_active_group))
    return false;
  if (!planSafeStateToPreGraspState(start_state, *pre_grasp_state, gs_active_group))
    return false;
  return true;
}

bool HybridMotionValidator::planNeedleReleasing(const robot_state::RobotState &handoff_state,
                                                const robot_state::RobotState &goal_state,
                                                const std::string &ss_active_group) const
{
  robot_state::RobotStatePtr ungrasped_state(new robot_state::RobotState(goal_state));

  planGraspStateToUngraspedState(handoff_state, ungrasped_state, ss_active_group);
  if (!planUngraspedStateToSafeState(*ungrasped_state, goal_state, ss_active_group))
    return false;
  return true;
}

bool HybridMotionValidator::planPreGraspStateToGraspedState(robot_state::RobotStatePtr& pre_grasp_state,
                                                            const robot_state::RobotState &handoff_state,
                                                            const std::string &planning_group) const
{
  // make a pre_grasp_state
  const robot_state::JointModelGroup *arm_joint_group = pre_grasp_state->getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d pregrasp_tool_tip_pose = grasped_tool_tip_pose;
  Eigen::Vector3d unit_approach_dir(0.0, 0.0, 1.0);  // grasp approach along the +z-axis of tip frame

  auto start_ik = std::chrono::high_resolution_clock::now();

  bool found_ik = false;
  double distance = 0.008;
  while (distance <= 0.013)
  {
    Eigen::Vector3d approach_dir = grasped_tool_tip_pose.linear() * (distance * unit_approach_dir);
    pregrasp_tool_tip_pose.translation() = grasped_tool_tip_pose.translation() - approach_dir;
    std::size_t attempts = 1;
    double timeout = 0.1;
    found_ik = pre_grasp_state->setFromIK(arm_joint_group, pregrasp_tool_tip_pose, attempts, timeout);
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

  std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  pre_grasp_state->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (std::size_t i = 0; i < eef_joint_position.size(); i++)
  {
    eef_joint_position[i] = 0.5;
  }
  pre_grasp_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  setMimicJointPositions(pre_grasp_state, planning_group);
  pre_grasp_state->update();

  // publishRobotState(*pre_grasp_state);

  std::vector<robot_state::RobotStatePtr> traj;
  double translation_step_max = 0.001, rotation_step_max = 0.0;
  moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
//  double jt_revolute = 0.0, jt_prismatic = 0.0, jump_threshold_factor = 0.1;
  moveit::core::JumpThreshold jump_threshold;
  double found_cartesian_path = pre_grasp_state->computeCartesianPath(arm_joint_group,
                                                                      traj,
                                                                      tip_link,
                                                                      grasped_tool_tip_pose,
                                                                      true,
                                                                      max_step,
                                                                      jump_threshold);

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (found_cartesian_path != 1.0)
  {
    return clear_path;
  }
  setMimicJointPositions(pre_grasp_state, planning_group);
  pre_grasp_state->update();

  // publishRobotState(*pre_grasp_state);

  const moveit::core::AttachedBody *hdof_needle_body = handoff_state.getAttachedBody(object_name_);

  traj.back()->attachBody(hdof_needle_body->getName(), hdof_needle_body->getShapes(),
                          hdof_needle_body->getFixedTransforms(), hdof_needle_body->getTouchLinks(),
                          hdof_needle_body->getAttachedLinkName(), hdof_needle_body->getDetachPosture());

  for (std::size_t i = 0; i < eef_joint_position.size(); i++)
  {
    eef_joint_position[i] = 0.0;
  }
  traj.back()->setJointGroupPositions(eef_group_name,eef_joint_position);
  traj.back()->update();

  for (std::size_t i = 0; i < traj.size(); i++)
  {
    setMimicJointPositions(traj[i], planning_group);
    traj[i]->update();
    // publishRobotState(*traj[i]);
    if (!noCollision(*traj[i]))  // check intermediate states
    {
      // publishRobotState(*traj[i]);
      return clear_path;
    }
  }
  pre_grasp_state.reset(new robot_state::RobotState(*traj[0]));
  pre_grasp_state->update();

  clear_path = true;
  return clear_path;
}


bool HybridMotionValidator::planSafeStateToPreGraspState(const robot_state::RobotState &start_state,
                                                         const robot_state::RobotState &pre_grasp_state,
                                                         const std::string &planning_group) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state(new robot_state::RobotState(start_state));
  const robot_state::JointModelGroup *arm_joint_group = pre_grasp_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d pre_grasp_tool_tip_pose = pre_grasp_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;
  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     pre_grasp_tool_tip_pose,
                                                                     true,
                                                                     0.001,
                                                                     0.0);
  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (found_cartesian_path != 1.0)
  {
    return clear_path;
  }

  // removable
  setMimicJointPositions(cp_start_state, planning_group);
  cp_start_state->update();
  // publishRobotState(*cp_start_state);

  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    setMimicJointPositions(traj[i], planning_group);
    traj[i]->update();
    // publishRobotState(*traj[i]);
    if (!noCollision(*traj[i]))
    {
      // publishRobotState(*traj[i]);
      return clear_path;
    }
  }
  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planGraspStateToUngraspedState(const robot_state::RobotState &handoff_state,
                                                           robot_state::RobotStatePtr& ungrasped_state,
                                                           const std::string &planning_group) const
{
  // make a ungrasped_state
  const robot_state::JointModelGroup *arm_joint_group = handoff_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d ungrasped_tool_tip_pose = grasped_tool_tip_pose;
  Eigen::Vector3d retreat_dir(0.0, 0.0, -1.0);  // ungrasp retreat along the -z-axis of tip frame
  double distance = 0.01;
  retreat_dir = grasped_tool_tip_pose.linear() * (distance * retreat_dir);
  ungrasped_tool_tip_pose.translation() += retreat_dir;

  auto start_ik = std::chrono::high_resolution_clock::now();

  std::size_t attempts = 1;
  double timeout = 0.1;
  bool found_ik = ungrasped_state->setFromIK(arm_joint_group, grasped_tool_tip_pose, attempts, timeout);

  if (!found_ik)
  {
    auto finish_ik = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish_ik - start_ik;
    hyStateSpace_->ik_solving_duration_ += elapsed;
    return found_ik;
  }

  std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  ungrasped_state->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (std::size_t i = 0; i < eef_joint_position.size(); i++)
  {
    eef_joint_position[i] = 0.5;
  }
  ungrasped_state->setJointGroupPositions(eef_group_name, eef_joint_position);
  setMimicJointPositions(ungrasped_state, planning_group);
  ungrasped_state->update();

  // publishRobotState(*ungrasped_state);

  std::vector<robot_state::RobotStatePtr> traj;
  double translation_step_max = 0.001, rotation_step_max = 0.0;
  moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);
//  double jt_revolute = 0.0, jt_prismatic = 0.0, jump_threshold_factor = 0.1;
  moveit::core::JumpThreshold jump_threshold;
  double found_cartesian_path = ungrasped_state->computeCartesianPath(arm_joint_group,
                                                                      traj,
                                                                      tip_link,
                                                                      ungrasped_tool_tip_pose,
                                                                      true,
                                                                      max_step,
                                                                      jump_threshold);


  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (found_cartesian_path != 1.0)
  {
    return clear_path;
  }

  ungrasped_state->setToDefaultValues(ungrasped_state->getJointModelGroup(eef_group_name), eef_group_name + "_home");

  for (std::size_t i = 0; i < traj.size(); i++)
  {
    setMimicJointPositions(traj[i], planning_group);
    traj[i]->update();
    // publishRobotState(*traj[i]);
    if (!noCollision(*traj[i]))  // check intermediate states
    {
      // publishRobotState(*traj[i]);
      if (i > 0)
      {
        ungrasped_state.reset(new robot_state::RobotState(*traj[i - 1]));
        ungrasped_state->update();
        clear_path = true;
        return clear_path;
      }
      return clear_path;
    }
  }
  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planUngraspedStateToSafeState(const robot_state::RobotState &ungrasped_state,
                                                          const robot_state::RobotState &goal_state,
                                                          const std::string &planning_group) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state(new robot_state::RobotState(ungrasped_state));
  const robot_state::JointModelGroup *arm_joint_group = goal_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;
  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     tool_tip_pose,
                                                                     true,
                                                                     0.001,
                                                                     0.0);
  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;


  bool clear_path = false;
  if (found_cartesian_path != 1.0)
  {
    return clear_path;
  }

  // removable
  setMimicJointPositions(cp_start_state, planning_group);
  cp_start_state->update();
  // publishRobotState(*cp_start_state);

  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    setMimicJointPositions(traj[i], planning_group);
    traj[i]->update();
    // publishRobotState(*traj[i]);
    if (!noCollision(*traj[i]))
    {
      // publishRobotState(*traj[i]);
      return clear_path;
    }
  }
  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::planObjectTransit(const robot_state::RobotState &start_state,
                                              const robot_state::RobotState &goal_state,
                                              const std::string &planning_group) const
{
  hyStateSpace_->object_transit_motion_planner_num += 1;
  auto start_ik = std::chrono::high_resolution_clock::now();

  const robot_state::RobotStatePtr cp_start_state(new robot_state::RobotState(start_state));
  const robot_state::JointModelGroup *arm_joint_group = goal_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d tool_tip_pose = goal_state.getGlobalLinkTransform(tip_link);

  std::vector<robot_state::RobotStatePtr> traj;
  double found_cartesian_path = cp_start_state->computeCartesianPath(cp_start_state->getJointModelGroup(planning_group),
                                                                     traj,
                                                                     tip_link,
                                                                     tool_tip_pose,
                                                                     true,
                                                                     0.001,
                                                                     0.0);
  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->ik_solving_duration_ += elapsed;

  bool clear_path = false;
  if (found_cartesian_path != 1.0)
  {
    return clear_path;
  }

  // removable
  setMimicJointPositions(cp_start_state, planning_group);
  cp_start_state->update();
  // publishRobotState(*cp_start_state);

  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    setMimicJointPositions(traj[i], planning_group);
    traj[i]->update();
    // publishRobotState(*traj[i]);
    if (!noCollision(*traj[i]))
    {
      // publishRobotState(*traj[i]);
      return clear_path;
    }
  }
  clear_path = true;
  return clear_path;
}

bool HybridMotionValidator::noCollision(const robot_state::RobotState& rstate) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  planning_scene_->setCurrentState(rstate);
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_detection::CollisionResult collision_result;
  planning_scene_->checkCollision(collision_request, collision_result, rstate);
  bool no_collision = !collision_result.collision;

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->collision_checking_duration_ += elapsed;

  if(collision_result.collision)
  {
    ROS_INFO("Invalid State: Robot state is in collision with planning scene. \n");
    collision_detection::CollisionResult::ContactMap contactMap = collision_result.contacts;
    for(collision_detection::CollisionResult::ContactMap::const_iterator it = contactMap.begin(); it != contactMap.end(); ++it)
    {
      ROS_INFO("Contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
    }
  }
  return no_collision;
}

void HybridMotionValidator::setMimicJointPositions(const robot_state::RobotStatePtr &rstate,
                                                   const std::string &planning_group) const
{
  const std::string outer_pitch_joint = (planning_group == "psm_one") ? "PSM1_outer_pitch" : "PSM2_outer_pitch";
  const double *joint_val = rstate->getJointPositions(outer_pitch_joint);
  if(joint_val)
    rstate->setJointGroupPositions(planning_group + "_base_mimics", joint_val);
}
