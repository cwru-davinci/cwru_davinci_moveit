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

namespace dual_arm_manipulation_planner_interface
{

HybridMotionValidator::HybridMotionValidator(const ros::NodeHandle &node_handle,
                                             const ros::NodeHandle &node_priv,
                                             const std::string &robot_name,
                                             const std::string &object_name,
                                             const ompl::base::SpaceInformationPtr &si) :
  ompl::base::MotionValidator(si), node_handle_(node_handle), robot_model_loader_(robot_name), robot_name_(robot_name),
  stateValidityChecker_(node_handle, node_priv, robot_name, object_name, si)
{
  kmodel_.reset(
    new robot_model::RobotModel(robot_model_loader_.getModel()->getURDF(), robot_model_loader_.getModel()->getSRDF()));

  initializePlannerPlugin();

  pMonitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_name_));
}

bool HybridMotionValidator::checkMotion (const ompl::base::State *s1, const ompl::base::State *s2) const
{
  if (!si_->isValid(s2))
  {
    invalid_++;
    return false;
  }

  bool result = true;

  const auto *hs1 = static_cast<const HybridObjectStateSpace::StateType *>(s1);
  const auto *hs2 = static_cast<const HybridObjectStateSpace::StateType *>(s2);

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
    {
      if(!planHandoff(group_s1, group_s2, object_pose, grasp_index))
        return false;
    }

    case StateDiff::PoseDiffArmAndGraspSame:
    {
      pMonitor_->requestPlanningSceneState();
      planning_scene_monitor::LockedPlanningSceneRO ls(pMonitor_);

      // convert from object_pose to robot's tool tip pose
      Eigen::Affine3d object_pose_s2;  // object pose w/rt base frame
      si_->getStateSpace()->as<HybridObjectStateSpace>()->se3ToEign3d(hs2, object_pose_s2);
      Eigen::Affine3d grasp_pose_s2 = si_->getStateSpace()->as<HybridObjectStateSpace>()->possible_grasps_[hs2->graspIndex().value].grasp_pose;
      geometry_msgs::PoseStamped tool_tip_pose_s2;  // tool tip pose w/rt base frame
      tool_tip_pose_s2.header.frame_id = ls->getPlanningFrame();
      tf::poseEigenToMsg(object_pose_s2 * grasp_pose_s2.inverse(), tool_tip_pose_s2.pose);

      // create a motion plan request
      planning_interface::MotionPlanRequest req;
      planning_interface::MotionPlanResponse res;

      req.group_name = group_s2;

      moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
        joint_model_group_s2->getOnlyOneEndEffectorTip()->getName(),
        tool_tip_pose_s2);
      req.goal_constraints.push_back(pose_goal);

      planning_interface::PlanningContextPtr context =
        planner_instance_->getPlanningContext(ls, req, res.error_code_);
      context->solve(res);
      if (res.error_code_.val != res.error_code_.SUCCESS)
      {
        ROS_ERROR("Could not compute plan successfully");
        return false;
      }
    }
    case StateDiff::ArmAndGraspDiffPoseSame:
      // Todo handoff operation check
      return false;
//    case StateDiff::GraspDiffArmAndPoseSame:
//      // Todo handoff operation check
//      break;
//    case StateDiff::ArmAndPoseDiffGraspSame:
//      // Todo handoff operation check
//      break;
//    case StateDiff::GraspAndPoseDiffArmSame:
//      // Todo handoff operation check
//      break;
//    case StateDiff::AllDiff:
//      // Todo handoff operation check
//      break;
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

bool HybridMotionValidator::planHandoff(const std::string &support_arm_from,
                                        const std::string &support_ar_to,
                                        const ompl::base::SE3StateSpace::StateType &object_pose,
                                        const int &grasp_index) const
{
  geometry_msgs::PoseStamped needle_pose;
  needle_pose.pose.position.x = object_pose.getX();
  needle_pose.pose.position.y = object_pose.getY();
  needle_pose.pose.position.z = object_pose.getZ();

  needle_pose.pose.orientation.x = object_pose.rotation().x;
  needle_pose.pose.orientation.y = object_pose.rotation().y;
  needle_pose.pose.orientation.z = object_pose.rotation().z;
  needle_pose.pose.orientation.w = object_pose.rotation().w;

  graspGeneratorHelper(const geometry_msgs::PoseStamped &needle_pose,
  const DavinciNeeldeGraspData &needleGraspData,
  std::vector<GraspInfo> &grasp_pose);





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