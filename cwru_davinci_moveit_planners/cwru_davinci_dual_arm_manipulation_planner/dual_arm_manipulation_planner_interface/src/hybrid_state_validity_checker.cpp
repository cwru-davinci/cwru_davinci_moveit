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
 * Description: The class to do state validity check
 */

#include <moveit/dual_arm_manipulation_planner_interface/hybrid_state_validity_checker.h>
#include <moveit/dual_arm_manipulation_planner_interface/threadsafe_state_storage.h>
#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

#include <moveit/move_group_interface/move_group_interface.h>

//#include <moveit/ompl_interface/ompl_planning_context.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>
#include <geometric_shapes/shape_operations.h>

#include <cwru_davinci_grasp/davinci_needle_grasp_data.h>

using namespace dual_arm_manipulation_planner_interface;
//using namespace davinci_moveit_object_handling;

HybridStateValidityChecker::HybridStateValidityChecker(const ros::NodeHandle &node_handle,
                                                       const ros::NodeHandle &node_priv,
                                                       const std::string &robot_name,
                                                       const std::string &object_name,
                                                       const ompl::base::SpaceInformationPtr &si)
  : node_handle_(node_handle), robot_model_loader_(robot_name), robot_name_(robot_name),object_name_(object_name),
    ompl::base::StateValidityChecker(si)
{

  kmodel_.reset(
    new robot_model::RobotModel(robot_model_loader_.getModel()->getURDF(), robot_model_loader_.getModel()->getSRDF()));

  planning_scene_.reset(new planning_scene::PlanningScene(kmodel_));

//  pMonitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_name_));

  complete_initial_robot_state_.reset(new robot_state::RobotState(kmodel_));

  tss_.reset(new TSStateStorage(*complete_initial_robot_state_));

  collision_request_with_distance_.distance = true;

  collision_request_with_cost_.cost = true;
}

bool HybridStateValidityChecker::isValid(const ompl::base::State *state) const
{
// check bounds
  if (!si_->satisfiesBounds(state))
  {
    return false;
  }

  // convert ompl state to moveit robot state
  robot_state::RobotState *kstate = tss_->getStateStorage();
  convertObjectToRobotState(*kstate, state);

//  pMonitor_->requestPlanningSceneState();
//  planning_scene_monitor::LockedPlanningSceneRW ls(pMonitor_);
//  ls->setCurrentState(*kstate);
//  // check path constraints
//  const kinematic_constraints::KinematicConstraintSetPtr &kset = planning_context_->getPathConstraints();
//  if (kset && !kset->decide(*kstate, verbose).satisfied)
//    return false;
  planning_scene_->setCurrentState(*kstate);
  // check feasibility
  if (!planning_scene_->isStateFeasible(*kstate))
    return false;

  // check collision avoidance
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_simple_, res, *kstate);
  return res.collision == false;
}

double HybridStateValidityChecker::cost(const ompl::base::State* state) const
{
  double cost = 0.0;

  robot_state::RobotState *kstate = tss_->getStateStorage();
  convertObjectToRobotState(*kstate, state);

//  pMonitor_->requestPlanningSceneState();
//  planning_scene_monitor::LockedPlanningSceneRW ls(pMonitor_);
//  ls->setCurrentState(*kstate);
  planning_scene_->setCurrentState(*kstate);
  // Calculates cost from a summation of distance to obstacles times the size of the obstacle
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_with_cost_, res, *kstate);

  for (std::set<collision_detection::CostSource>::const_iterator it = res.cost_sources.begin() ; it != res.cost_sources.end() ; ++it)
    cost += it->cost * it->getVolume();

  return cost;
}


double HybridStateValidityChecker::clearance(const ompl::base::State* state) const
{
  robot_state::RobotState *kstate = tss_->getStateStorage();
  convertObjectToRobotState(*kstate, state);

//  pMonitor_->requestPlanningSceneState();
//  planning_scene_monitor::LockedPlanningSceneRW ls(pMonitor_);
//  ls->setCurrentState(*kstate);
  planning_scene_->setCurrentState(*kstate);
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_with_distance_, res, *kstate);
  return res.collision ? 0.0 : (res.distance < 0.0 ? std::numeric_limits<double>::infinity() : res.distance);
}


void HybridStateValidityChecker::convertObjectToRobotState(robot_state::RobotState &rstate, const ompl::base::State *state) const
{
  const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(state);
  const std::string selected_group_name = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";
  const std::string rest_group_name = (selected_group_name == "psm_one") ? "psm_two" : "psm_one";
//  if(!hasAttachedObject(group_name, object_name_))  // check if "psm_one" holds the needle
//  {
//    group_name = (hs->armIndex().value == 1) ? "psm_two" : "psm_one";
//
//    if(!hasAttachedObject(group_name, object_name_))  // check if "psm_two" holds the needle
//    {
//      ROS_INFO("No arm is holding the object");
//      return;
//    }
//
//    group_name = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";  // reset group_name_ to correct value
//  }


//  pMonitor_->requestPlanningSceneState();
//  planning_scene_monitor::LockedPlanningSceneRO ls(pMonitor_);
//  rstate = ls->getCurrentState();

  // convert object pose to robot tip pose
  // this is the gripper tool tip link frame wrt /base_link
  Eigen::Affine3d object_pose;  // object pose w/rt base frame
  si_->getStateSpace()->as<HybridObjectStateSpace>()->se3ToEign3d(hs, object_pose);

  Eigen::Affine3d grasp_pose = si_->getStateSpace()->as<HybridObjectStateSpace>()->possible_grasps_[hs->graspIndex().value].grasp_pose;
  Eigen::Affine3d tool_tip_pose = object_pose * grasp_pose.inverse();

  const robot_state::JointModelGroup* selected_joint_model_group = rstate.getJointModelGroup(selected_group_name);
  std::size_t attempts = 10;
  double timeout = 0.1;
  bool found_ik = rstate.setFromIK(selected_joint_model_group, tool_tip_pose, attempts, timeout);

  if (found_ik)
  {
    const robot_state::JointModelGroup* rest_joint_model_group = rstate.getJointModelGroup(rest_group_name);
    rstate.setToDefaultValues(rest_joint_model_group, rest_group_name+"_home");

    std::string rest_group_eef_name = rest_joint_model_group->getAttachedEndEffectorNames()[0];
    const robot_state::JointModelGroup* rest_joint_model_group_eef = rstate.getJointModelGroup(rest_group_eef_name);
    rstate.setToDefaultValues(rest_joint_model_group_eef, rest_group_eef_name+"_home");

    rstate.attachBody(createAttachedBody(selected_joint_model_group, object_name_, grasp_pose).get());
    rstate.clearAttachedBody(object_name_);
    rstate.update();
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
}

std::unique_ptr<moveit::core::AttachedBody>
HybridStateValidityChecker::createAttachedBody(const robot_state::JointModelGroup *joint_model_group,
                                               const std::string &object_name,
                                               const Eigen::Affine3d &attach_tran) const
{
  const moveit::core::LinkModel *tip_link = joint_model_group->getOnlyOneEndEffectorTip();
  Eigen::Vector3d scale_vec(0.025, 0.025, 0.025);
  shapes::Mesh *needle_mesh =
    shapes::createMeshFromResource("package://cwru_davinci_geometry_models/"
                                     "props/needle_r/mesh/needle_r4.dae",
                                   scale_vec);
  std::vector<shapes::ShapeConstPtr> shapes = {needle_mesh};
  EigenSTL::vector_Affine3d attach_trans = {attach_tran};

  const robot_state::JointModelGroup *eef_group = kmodel_->getJointModelGroup(
    joint_model_group->getAttachedEndEffectorNames()[0]);

  std::vector<std::string> touch_links_list = eef_group->getLinkModelNames();
  std::set<std::string> touch_links(touch_links_list.begin(), touch_links_list.end());

  trajectory_msgs::JointTrajectory grasp_jt;
  grasp_jt.header.stamp = ros::Time::now();
  grasp_jt.header.frame_id = kmodel_->getModelFrame().c_str();
  grasp_jt.joint_names = eef_group->getVariableNames();
  grasp_jt.points.resize(3);
  grasp_jt.points[0].positions.push_back(-1.0);
  grasp_jt.points[1].positions.push_back(-1.0);
  grasp_jt.points[2].positions.push_back(-1.0);

  return std::unique_ptr<moveit::core::AttachedBody>(
    new moveit::core::AttachedBody(tip_link, object_name, shapes, attach_trans, touch_links, grasp_jt));
}


//bool HybridStateValidityChecker::hasAttachedObject(const std::string& group_name, const std::string& object_name) const
//{
//
//  moveit::planning_interface::PlanningSceneInterface ps_interface;
//  std::map<std::string, moveit_msgs::AttachedCollisionObject> objs = ps_interface.getAttachedObjects();
//
//  std::map<std::string, moveit_msgs::AttachedCollisionObject>::iterator existing = objs.find(object_name);
//  if(existing != objs.end())
//  {
//    moveit_msgs::AttachedCollisionObject attached_obj = existing->second;
//
//    std::string link_name = attached_obj.link_name;
//
//    moveit::planning_interface::MoveGroupInterface move_group(group_name);
//    std::string end_effector = move_group.getEndEffector();
//    moveit::planning_interface::MoveGroupInterface eff_group(end_effector);
//    std::vector<std::string> eff_links = eff_group.getLinkNames();
//    std::vector<std::string>::iterator it;
//
//    it = std::find(eff_links.begin(), eff_links.end(), link_name);
//    if(it != eff_links.end())
//    {
//      return true;
//    }
//    return false;
//  }
//  return false;
//}