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
#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

#include <moveit/move_group_interface/move_group_interface.h>

//#include <moveit/ompl_interface/ompl_planning_context.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>


using namespace dual_arm_manipulation_planner_interface;
//using namespace davinci_moveit_object_handling;

HybridStateValidityChecker::HybridStateValidityChecker(const ros::NodeHandle &node_handle,
                                           const ros::NodeHandle &node_priv,
                                           const std::string &robot_name,
                                           const std::string &object_name,
                                           const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
                                           const ompl::base::SpaceInformationPtr &si)
  : node_handle_(node_handle), robot_model_loader_(robot_name), object_name_(object_name), possible_grasps_(possible_grasps),
    ompl::base::StateValidityChecker(si)
{

  kmodel_.reset(
    new robot_model::RobotModel(robot_model_loader_.getModel()->getURDF(), robot_model_loader_.getModel()->getSRDF()));

  planning_scene_.reset(new planning_scene::PlanningScene(kmodel_));

  complete_initial_robot_state_.reset(new robot_state::RobotState(kmodel_));

  tss_(complete_initial_robot_state_);

  initializePlannerPlugin();

  collision_request_simple_verbose_ = collision_request_simple_;
  collision_request_simple_verbose_.verbose = true;
}


void HybridStateValidityChecker::setVerbose(bool flag)
{
  verbose_ = flag;
}

double HybridStateValidityChecker::cost(const ompl::base::State* state) const
{
  double cost = 0.0;

  robot_state::RobotState *kstate = tss_.getStateStorage();
  convertObjectToRobotState(kstate, state);

  // Calculates cost from a summation of distance to obstacles times the size of the obstacle
  collision_detection::CollisionResult res;
  planning_context_->getPlanningScene()->checkCollision(collision_request_with_cost_, res, *kstate);

  for (std::set<collision_detection::CostSource>::const_iterator it = res.cost_sources.begin() ; it != res.cost_sources.end() ; ++it)
    cost += it->cost * it->getVolume();

  return cost;
}


double HybridStateValidityChecker::clearance(const ompl::base::State* state) const
{

}

bool HybridStateValidityChecker::isValidWithoutCache(const ompl::base::State *state, bool verbose) const
{

}

bool HybridStateValidityChecker::isValidWithCache(const ompl::base::State* state, bool verbose) const
{
  const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(state);

  group_name_ = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";

  robot_state::RobotState* kstate = tss_.getStateStorage();


}

void HybridStateValidityChecker::convertObjectToRobotState(robot_state::RobotState* robot_state, const ompl::base::State* state)
{
  const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(state);

  group_name_ = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";

  if(!hasAttachedObject(group_name_, object_name_))  // check if "psm_one" holds the needle
  {
    group_name_ = (hs->armIndex().value == 1) ? "psm_two" : "psm_one";

    if(!hasAttachedObject(group_name_, object_name_))  // check if "psm_two" holds the needle
    {
      ROS_INFO("No arm is holding the object");
      return;
    }

    group_name_ = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";  // reset group_name_ to correct value
  }

  cwru_davinci_grasp::GraspInfo grasp_info = possible_grasps_[hs->graspIndex().value];
  geometry_msgs::PoseStamped grasp_pose = grasp_info.grasp.grasp_pose;  // this is the gripper tool tip link frame wrt /base_link

  Eigen::Affine3d tool_tip_pose;

  tf::poseMsgToEigen(grasp_pose.pose, tool_tip_pose);

  const robot_state::JointModelGroup* joint_model_group = kmodel_->getJointModelGroup(group_name_);

  std::size_t attempts = 10;
  double timeout = 0.1;
  bool found_ik = robot_state->setFromIK(joint_model_group, tool_tip_pose, attempts, timeout);

  if (found_ik)
  {
    robot_state->update();
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }
}

void HybridStateValidityChecker::initializePlannerPlugin()
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

bool HybridStateValidityChecker::hasAttachedObject(const std::string& group_name, const std::string& object_name)
{
  std::map<std::string, moveit_msgs::AttachedCollisionObject> objs = planning_scene_interface_->getAttachedObjects();

  std::map<std::string, moveit_msgs::AttachedCollisionObject>::iterator existing = objs.find(object_name);
  if(existing != objs.end())
  {
    moveit_msgs::AttachedCollisionObject attached_obj = existing->second;

    std::string link_name = attached_obj.link_name;

    moveit::planning_interface::MoveGroupInterface move_group(group_name);
    std::string end_effector = move_group.getEndEffector();
    moveit::planning_interface::MoveGroupInterface eff_group(end_effector);
    std::vector<std::string> eff_links = eff_group.getLinkNames();
    std::vector<std::string>::iterator it;

    it = std::find(eff_links.begin(), eff_links.end(), link_name);
    if(it != eff_links.end())
    {
      return true;
    }
    return false;
  }
  return false;
}
