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

/* Author: Su Lu */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <boost/scoped_ptr.hpp>

/**
 * method to deine a tool tip goal wrt planning frame
 * @param current_state the current robot state
 * @param goal_wrt_current the transform of goal pose wrt current pose
 * @param goal_wrt_planning_frame the transform of goal pose wrt planning frame
 */
void defineToolTipGoal(const moveit::core::RobotStatePtr &current_state,
                       const Eigen::Matrix3d &orientation,
                       const Eigen::Vector3d &translation,
                       Eigen::Affine3d &goal_wrt_planning_frame)
{

  Eigen::Affine3d goal_wrt_current;

  goal_wrt_current.linear() = orientation;

  goal_wrt_current.translation() = translation;

//  current_state->setToDefaultValues();
//  current_state->updateLinkTransforms();

  Eigen::Affine3d current_wrt_planning_frame = current_state->getFrameTransform(
    "/PSM1tool_tip_link");
  ROS_INFO_STREAM("current_wrt_planning_frame \n" << current_wrt_planning_frame.matrix());


  Eigen::Affine3d base_wrt_world = current_state->getFrameTransform(
    "/PSM1psm_base_link");

//  ROS_INFO("The end effector link is %s", move_group.getEndEffectorLink().c_str());

  goal_wrt_planning_frame = current_wrt_planning_frame * goal_wrt_current;
  ROS_INFO_STREAM("goal_wrt_planning_frame \n" << goal_wrt_planning_frame.matrix());
//  Eigen::Affine3d goal_pose_wrt_base =
//    base_wrt_world.inverse() * goal_pose_wrt_world;
//  ROS_INFO_STREAM("\n" << psm_1_tool_tip_goal_pose_wrt_base.matrix());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  ros::Duration(4.0).sleep();
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt! and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects, a RobotModel
  // and a PlanningScene.
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  if(!node_handle.getParam("planning_plugin", planner_plugin_name))
  {
    ROS_FATAL_STREAM("Could not find planner plugin name");
  }
  try
  {
    planner_plugin_loader.reset(
      new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));



  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
    {
      ROS_FATAL_STREAM("Could not initialize planner instance");
    }
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
    {
      ss << classes[i] << " ";
    }
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }


  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the right arm of the PR2
  // specifying the desired pose of the end-effector as input.

  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;

  pose.header.frame_id = "PSM1psm_base_link";
  pose.pose.position.x = -0.02;
  pose.pose.position.y = -0.01;
  pose.pose.position.z = 0.7;
  pose.pose.orientation.w = 0.0;
  pose.pose.orientation.x = -0.7071;
  pose.pose.orientation.y = 0.7071;
  pose.pose.orientation.z = 0.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // `kinematic_constraints`_
  // package.
  //
  // .. _kinematic_constraints: http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "psm_one";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("PSM1tool_tip_link", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context = planner_instance->getPlanningContext(planning_scene, req,
                                                                                             res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
    node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);

  sleep_time.sleep();
  return 0;
}
