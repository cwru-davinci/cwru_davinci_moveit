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
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

namespace dual_arm_manipulation_planner_interface
{

HybridMotionValidator::HybridMotionValidator(const ros::NodeHandle &node_handle,
                                             const ros::NodeHandle &node_priv,
                                             const std::string &robot_name,
                                             const std::string &object_name,
                                             const ompl::base::SpaceInformationPtr &si) :
  ompl::base::MotionValidator(si),
  node_handle_(node_handle),
  node_priv_(node_priv),
  robot_model_loader_(robot_name),
  robot_name_(robot_name),
  object_name_(object_name),
  stateValidityChecker_(node_handle, node_priv, robot_name, object_name, si)
{
  kmodel_.reset(
    new robot_model::RobotModel(robot_model_loader_.getModel()->getURDF(), robot_model_loader_.getModel()->getSRDF()));

  initializePlannerPlugin();

  planning_scene_.reset(new planning_scene::PlanningScene(kmodel_));

//  pMonitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_name_));
}

bool HybridMotionValidator::checkMotion (const ompl::base::State *s1, const ompl::base::State *s2) const
{
  if (!si_->isValid(s2) && !si_->isValid(s1))
  {
    invalid_++;
    return false;
  }

  bool result = true;


////  si_->getStateSpace()->as<HybridObjectStateSpace>()->interpolate(s1, s2, t, cstate);
//
//
//  std::string rest_group_s1 = (active_group_s1 == "psm_one") ? "psm_two" : "psm_one";
//  std::string rest_group_s2 = (active_group_s2 == "psm_one") ? "psm_two" : "psm_one";
//
//  const robot_state::JointModelGroup* joint_model_group_s1 = kmodel_->getJointModelGroup(group_s1);
//  const robot_state::JointModelGroup* joint_model_group_s2 = kmodel_->getJointModelGroup(group_s2);

  const auto *hs1 = static_cast<const HybridObjectStateSpace::StateType *>(s1);
  const auto *hs2 = static_cast<const HybridObjectStateSpace::StateType *>(s2);

  std::string active_group_s1 = (hs1->graspIndex().value == 1) ?  "psm_one" : "psm_two";  // active group is the arm supporting the needle
  std::string active_group_s2 = (hs2->graspIndex().value == 1) ?  "psm_one" : "psm_two";

  robot_state::RobotStatePtr start_state;
  start_state.reset(new robot_state::RobotState(planning_scene_->getCurrentState()));

  robot_state::RobotStatePtr goal_state;
  goal_state.reset(new robot_state::RobotState(planning_scene_->getCurrentState()));

  stateValidityChecker_.convertObjectToRobotState(*start_state, s1);
  stateValidityChecker_.convertObjectToRobotState(*goal_state, s2);
  switch(si_->getStateSpace()->as<HybridObjectStateSpace>()->checkStateDiff(hs1, hs2))
  {
    case StateDiff::AllSame:
      return true;
    case StateDiff::PoseDiffArmAndGraspSame:
    {

      return planPathFromTwoStates(*start_state, *goal_state, active_group_s1);
    }
    case StateDiff::ArmAndGraspDiffPoseSame:
    {
      if(!planHandoff(*start_state, *goal_state, active_group_s1, active_group_s2))
        return false;
    }
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

void HybridMotionValidator::initializePlannerPlugin()
{
  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle_.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
      "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance_.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance_->initialize(kmodel_, node_handle_.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance_->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }
}

bool HybridMotionValidator::planHandoff(const robot_state::RobotState &start_state,
                                        const robot_state::RobotState &goal_state,
                                        const std::string &ss_active_group,
                                        const std::string &gs_active_group) const
{
  bool able_to_grasp = false;
  bool able_to_release = false;

  robot_state::RobotStatePtr handoff_state;  // an intermediate state when needle is supporting by two grippers
  handoff_state.reset(new robot_state::RobotState(start_state));

  std::vector<double> gs_jt_position;
  goal_state.copyJointGroupPositions(gs_active_group, gs_jt_position);
  handoff_state->setJointGroupPositions(gs_active_group, gs_jt_position);

  if(!goal_state.hasAttachedBody(object_name_))
  {
    return false;
  }

  std::unique_ptr<moveit::core::AttachedBody> needle = stateValidityChecker_.createAttachedBody(gs_active_group, *handoff_state, object_name_,);

  handoff_state->attachBody((stateValidityChecker_.createAttachedBody(gs_active_group).get());

  able_to_grasp = planNeedleGrasping(start_state, *handoff_state, gs_active_group);
//
  able_to_release = planNeedleReleasing(*handoff_state, goal_state, ss_active_group);

  if(able_to_grasp && able_to_release)
    return true;
  else
    return false;
}

bool HybridMotionValidator::planNeedleGrasping(const robot_state::RobotState &start_state,
                                               const robot_state::RobotState &goal_state,
                                               const std::string &gs_active_group) const
{
  // planning in a back order fashion
  robot_state::RobotStatePtr pre_grasp_state;
  pre_grasp_state.reset(new robot_state::RobotState(start_state));
  pre_grasp_state->clearAttachedBody(object_name_);
  if (!planPreGraspStateToGraspedState(*pre_grasp_state, goal_state, gs_active_group))
    return false;

  if (!planSafeStateToPreGraspState(start_state, *pre_grasp_state, gs_active_group))
    return false;
  return true;

}

bool HybridMotionValidator::planNeedleReleasing(const robot_state::RobotState &start_state,
                                                const robot_state::RobotState &goal_state,
                                                const std::string &ss_active_group) const
{
  robot_state::RobotStatePtr ungrasped_state;
  ungrasped_state.reset(new robot_state::RobotState(start_state));
  ungrasped_state->clearAttachedBody(object_name_);

  if(!planGraspStateToUngraspedState(start_state, *ungrasped_state, ss_active_group))
    return false;
  if(!planUngraspedStateToSafeState(*ungrasped_state, goal_state, ss_active_group))
    return false;
  return true;
}

bool HybridMotionValidator::planPreGraspStateToGraspedState(robot_state::RobotState &pre_grasp_state,
                                                            const robot_state::RobotState &handoff_state,
                                                            const std::string &planning_group) const
{
  // make a pre_grasp_state
  const robot_state::JointModelGroup *arm_joint_group = handoff_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d pregrasp_tool_tip_pose = grasped_tool_tip_pose;
  pregrasp_tool_tip_pose.translation().z() -= 0.005;

  std::size_t attempts = 10;
  double timeout = 0.1;
  bool found_ik = pre_grasp_state.setFromIK(arm_joint_group, pregrasp_tool_tip_pose, attempts, timeout);

  if (!found_ik)
    return false;

  std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  pre_grasp_state.copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (int i = 0; i < eef_joint_position.size(); i++)
  {
    eef_joint_position[i] = 1.0;
  }
  pre_grasp_state.setJointGroupPositions(eef_group_name, eef_joint_position);
  pre_grasp_state.update();

  std::vector<robot_state::RobotStatePtr> traj;

  double translation_step_max = 0.001, rotation_step_max = 0.0;
  moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);

  double jt_revolute = 0.001, jt_prismatic = 0.01;
  moveit::core::JumpThreshold jump_threshold(jt_revolute, jt_prismatic);
  bool found_cartesian_path = pre_grasp_state.computeCartesianPath(arm_joint_group, traj, tip_link,
                                                                   grasped_tool_tip_pose, true, max_step,
                                                                   jump_threshold);
  if (!found_cartesian_path)
    return false;

  // check each state in order.
  if (!planPathFromTwoStates(pre_grasp_state, *traj[0], planning_group)) // check first segment
    return false;
  for (int i = 0; i < traj.size(); i++)
  {
    if (!planPathFromTwoStates(*traj[i], *traj[i + 1], planning_group))  // check intermediate states
      return false;
  }
  if (!planPathFromTwoStates(*traj.back(), handoff_state, planning_group + "gripper"))  // // check last two states
    return false;

  return true;
}


bool HybridMotionValidator::planSafeStateToPreGraspState(const robot_state::RobotState &start_state,
                                                         const robot_state::RobotState &pre_grasp_state,
                                                         const std::string &planning_group) const
{
  return planPathFromTwoStates(start_state, pre_grasp_state, planning_group);
}

bool HybridMotionValidator::planGraspStateToUngraspedState(const robot_state::RobotState &handoff_state,
                                                           robot_state::RobotState &ungrasped_state,
                                                           const std::string &planning_group) const
{
  // make a ungrasped_state
  const robot_state::JointModelGroup *arm_joint_group = handoff_state.getJointModelGroup(planning_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d grasped_tool_tip_pose = handoff_state.getGlobalLinkTransform(tip_link);
  Eigen::Affine3d ungrasped_tool_tip_pose = grasped_tool_tip_pose;
  ungrasped_tool_tip_pose.translation().z() += 0.005;

  std::size_t attempts = 10;
  double timeout = 0.1;
  bool found_ik = ungrasped_state.setFromIK(arm_joint_group, ungrasped_tool_tip_pose, attempts, timeout);

  if (!found_ik)
    return false;

  std::string eef_group_name = arm_joint_group->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  ungrasped_state.copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (int i = 0; i < eef_joint_position.size(); i++)
  {
    eef_joint_position[i] = 0.0;
  }
  ungrasped_state.setJointGroupPositions(eef_group_name, eef_joint_position);
  ungrasped_state.update();

  std::vector<robot_state::RobotStatePtr> traj;

  double translation_step_max = 0.001, rotation_step_max = 0.0;
  moveit::core::MaxEEFStep max_step(translation_step_max, rotation_step_max);

  double jt_revolute = 0.001, jt_prismatic = 0.01;
  moveit::core::JumpThreshold jump_threshold(jt_revolute, jt_prismatic);
  bool found_cartesian_path = ungrasped_state.computeCartesianPath(arm_joint_group, traj, tip_link,
                                                                   ungrasped_tool_tip_pose, true, max_step,
                                                                   jump_threshold);
  if (!found_cartesian_path)
    return false;

  // check each state in order.
  if (!planPathFromTwoStates(handoff_state, *traj[0], planning_group)) // check first segment
    return false;
  for (int i = 0; i < traj.size(); i++)
  {
    if (!planPathFromTwoStates(*traj[i], *traj[i + 1], planning_group))  // check intermediate states
      return false;
  }
  if (!planPathFromTwoStates(*traj.back(), ungrasped_state, planning_group + "gripper"))  // // check last two states
    return false;

  return true;
}

bool HybridMotionValidator::planUngraspedStateToSafeState(const robot_state::RobotState &ungrasped_state,
                                                          const robot_state::RobotState &goal_state,
                                                          const std::string &planning_group) const
{
  return planPathFromTwoStates(ungrasped_state, goal_state, planning_group);
}

bool HybridMotionValidator::planPathFromTwoStates(const robot_state::RobotState &start_state,
                                                  const robot_state::RobotState &goal_state,
                                                  const std::string &planning_group) const
{

  planning_scene_->setCurrentState(start_state);

  const robot_state::JointModelGroup *joint_model_group = goal_state.getJointModelGroup(planning_group);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  // create a motion plan request
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;

  moveit_msgs::RobotState start_state_msg;
  moveit::core::robotStateToRobotStateMsg(start_state, start_state_msg);

  req.start_state = start_state_msg;
  req.group_name = planning_group;
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // But, let's impose a path constraint on the motion.
  // Here, we are asking for the end-effector to stay level

//  geometry_msgs::QuaternionStamped quaternion;
//  quaternion.header.frame_id = planning_scene_->getPlanningFrame();
//  quaternion.quaternion.w = 1.0;
//  req.path_constraints = kinematic_constraints::constructGoalConstraints(
//    joint_model_group->getOnlyOneEndEffectorTip()->getName(), quaternion);

  planning_interface::PlanningContextPtr context = planner_instance_->getPlanningContext(planning_scene_,
                                                                                         req,
                                                                                         res.error_code_);
  if(context->solve(res))
  {
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);
    return true;
  }
  else
    return false;
}

//bool ompl::base::DiscreteMotionValidator::checkMotion(const State *s1, const State *s2) const
//{
//  /* assume motion starts in a valid configuration so s1 is valid */
//  if (!si_->isValid(s2))
//  {
//    invalid_++;
//    return false;
//  }
//
//  bool result = true;
//  int nd = stateSpace_->validSegmentCount(s1, s2);
//
//  /* initialize the queue of test positions */
//  std::queue<std::pair<int, int>> pos;
//  if (nd >= 2)
//  {
//    pos.push(std::make_pair(1, nd - 1));
//
//    /* temporary storage for the checked state */
//    State *test = si_->allocState();
//
//    /* repeatedly subdivide the path segment in the middle (and check the middle) */
//    while (!pos.empty())
//    {
//      std::pair<int, int> x = pos.front();
//
//      int mid = (x.first + x.second) / 2;
//      stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, test);
//
//      if (!si_->isValid(test))
//      {
//        result = false;
//        break;
//      }
//
//      pos.pop();
//
//      if (x.first < mid)
//        pos.push(std::make_pair(x.first, mid - 1));
//      if (x.second > mid)
//        pos.push(std::make_pair(mid + 1, x.second));
//    }
//
//    si_->freeState(test);
//  }
//
//  if (result)
//    valid_++;
//  else
//    invalid_++;
//
//  return result;
//}

}  // namespace