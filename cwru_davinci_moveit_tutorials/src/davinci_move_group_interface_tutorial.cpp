/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Case Western resize University
 *  All rights resized.
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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>



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

//  current_state = move_group.getCurrentState();
  current_state->updateLinkTransforms();

  Eigen::Affine3d current_wrt_planning_frame = current_state->getFrameTransform(
    "/PSM1_tool_tip_link");
  ROS_INFO_STREAM("current_wrt_planning_frame \n" << current_wrt_planning_frame.matrix());


//  Eigen::Affine3d base_wrt_world = current_state->getFrameTransform(
//    "/one_psm_base_link");

//  ROS_INFO("The end effector link is %s", move_group.getEndEffectorLink().c_str());

  goal_wrt_planning_frame = current_wrt_planning_frame * goal_wrt_current;
  ROS_INFO_STREAM("goal_wrt_planning_frame \n" << goal_wrt_planning_frame.matrix());
//  Eigen::Affine3d goal_pose_wrt_base =
//    base_wrt_world.inverse() * goal_pose_wrt_world;
//  ROS_INFO_STREAM("\n" << psm_1_tool_tip_goal_pose_wrt_base.matrix());
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  ros::Duration(4.0).sleep();
  spinner.start();

  //***********************************************************************************************************

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
  ROS_INFO_NAMED("tutorial", "Frame in which the transforms for this model are computed: %s",
                 robot_model->getModelFrame().c_str());

  moveit::core::RobotStatePtr daVinciRobotState(new robot_state::RobotState(robot_model));
  daVinciRobotState->setToDefaultValues();

  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
//    ROS_INFO_STREAM("\n" << planning_scene->getFrameTransform("/psm_one_tool_wrist_sca_shaft_link").matrix());
//    ROS_INFO_STREAM("\n" << daVinciRobotState->getFrameTransform("/psm_one_tool_wrist_sca_shaft_link").matrix());

  //***********************************************************************************************************

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PSM_ONE_PLANNING_GROUP = "psm_one";
  static const std::string PSM_TWO_PLANNING_GROUP = "psm_two";

  const robot_state::JointModelGroup* psm1_jt_group_parent = daVinciRobotState->getJointModelGroup(PSM_ONE_PLANNING_GROUP);
  std::vector<std::string> sub_group_name = psm1_jt_group_parent->getSubgroupNames();
  const robot_state::JointModelGroup* eff_group = daVinciRobotState->getJointModelGroup(psm1_jt_group_parent->getAttachedEndEffectorNames()[0]);
  std::vector<std::string> eff_links = eff_group->getLinkModelNames();
  std::vector< const moveit::core::LinkModel * > tips;
  bool xx = psm1_jt_group_parent->getEndEffectorTips(tips);

  const moveit::core::LinkModel* tip_link = psm1_jt_group_parent->getOnlyOneEndEffectorTip();
  std::string tip_link_name = tip_link->getName();

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PSM_ONE_PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *psm1_jt_group =
    move_group.getCurrentState()->getJointModelGroup(PSM_ONE_PLANNING_GROUP);

  daVinciRobotState = move_group.getCurrentState();

  robot_state::RobotState dual_arm_state = planning_scene->getCurrentState();

  psm1_jt_group = dual_arm_state.getJointModelGroup(PSM_ONE_PLANNING_GROUP);

  const robot_state::JointModelGroup *psm2_jt_group = dual_arm_state.getJointModelGroup(PSM_TWO_PLANNING_GROUP);

  // Next get the current set of joint values for the group.
  std::vector<double> psm1_jt_pos;
  dual_arm_state.copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);

  ROS_INFO("Initial daVinciRobotState joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  std::vector<double> psm2_jt_pos;
  dual_arm_state.copyJointGroupPositions(psm2_jt_group, psm2_jt_pos);

  ROS_INFO("Initial daVinciRobotState joint values: ");
  for(int i = 0; i < psm2_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm2_jt_pos[i]);
  }

  std::vector< double > position1;
  position1.push_back(0.0185);
  dual_arm_state.setJointPositions("PSM1_outer_insertion", position1);

  planning_scene->setCurrentState(dual_arm_state);

  psm1_jt_pos.resize(0);
  dual_arm_state.copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);
  ROS_INFO("dual_arm_state joint values after the change of PSM1_outer_insertion: PSM1");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  dual_arm_state.copyJointGroupPositions(psm2_jt_group, psm2_jt_pos);

  ROS_INFO("dual_arm_state joint values after the change of PSM1_outer_insertion: PSM2");
  for(int i = 0; i < psm2_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm2_jt_pos[i]);
  }


  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in Rviz as well as debugging tools such as step-by-step introspection of a script
//  namespace rvt = rviz_visual_tools;
//  moveit_visual_tools::MoveItVisualTools visual_tools("world");
//  visual_tools.deleteAllMarkers();
//
//  // Remote control is an introspection tool that allows users to step through a high level script
//  // via buttons and keyboard shortcuts in Rviz
//  visual_tools.loadRemoteControl();
//
//  // Rviz provides many types of markers, in this demo we will use text, cylinders, and spheres
//  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
//
//  text_pose.translation().z() = 1.75;  // a magic number need to be changed later
//
//  visual_tools.publishText(text_pose, "daVinciMoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
//
//  // Batch publishing is used to reduce the number of messages being sent to Rviz for large visualizations
//  visual_tools.trigger();
//
//  // Getting Basic Information
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^
//  //
//  // We can print the name of the reference frame for this robot.
//  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
//
//  // We can also print the name of the end-effector link for this group.
//  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
//
//  ROS_INFO_NAMED("tutorial", "End effector name: %s", move_group.getEndEffector().c_str());

  // get camera to world transformation
  Eigen::Affine3d camera_to_world_transfom = daVinciRobotState->getFrameTransform("/camera");


  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  //**************************************************************************************************************

  Eigen::Affine3d target_pose_1;

  Eigen::Matrix3d orientation;  // orientation part
  orientation << 1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0;

  Eigen::Vector3d translation;  // translation part
  translation << 0.0, 0.0, 0.1;

  daVinciRobotState = move_group.getCurrentState();

  defineToolTipGoal(daVinciRobotState, orientation, translation, target_pose_1);

  move_group.setPoseTarget(target_pose_1);

  //**************************************************************************************************************
//    geometry_msgs::Pose target_pose_1;
//    target_pose_1.orientation.w = 1;
//    target_pose_1.orientation.x = -0.7071;
//    target_pose_1.orientation.y = 0;
//    target_pose_1.orientation.z = 0;
//
//
//    target_pose_1.position.x = 0.0;
//    target_pose_1.position.y = 0.05;
//    target_pose_1.position.z = 0.0;
//    move_group.setPoseTarget(target_pose_1);
//    move_group.setPoseReferenceFrame("psm_one_tool_wrist_sca_shaft_link");

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (bool) move_group.plan(my_plan);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  daVinciRobotState->setFromIK(psm1_jt_group, target_pose_1, 10, 0.1);
  daVinciRobotState->update();
  psm1_jt_pos.resize(0);
  daVinciRobotState->copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);

  ROS_INFO("after setFrom IK daVinciRobotState joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  planning_scene_monitor::PlanningSceneMonitorPtr pMonitor;
  pMonitor.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  pMonitor->requestPlanningSceneState();
  planning_scene_monitor::LockedPlanningSceneRW ls(pMonitor);

  robot_state::RobotState& current_state = ls->getCurrentStateNonConst();
  psm1_jt_pos.resize(0);
  current_state.copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);

  ROS_INFO("before setting current joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }


  ls->setCurrentState(*daVinciRobotState);
  current_state = ls->getCurrentStateNonConst();
  psm1_jt_pos.resize(0);
  current_state.copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);

  ROS_INFO("after setting current joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  daVinciRobotState = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  psm1_jt_pos.resize(0);
  daVinciRobotState->copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);

  ROS_INFO("after first planning daVinciRobotState joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  daVinciRobotState = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  psm1_jt_pos.resize(0);
  daVinciRobotState->copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);

  ROS_INFO("after press reset daVinciRobotState joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }
  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.
//  visual_tools.publishAxisLabeled(target_pose_1, "pose1");
//  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, psm1_jt_group);
//  visual_tools.trigger();
//  moveit_msgs::MoveItErrorCodes error_code = move_group.move();
//  if (error_code.val == 1)
//  {
//    ROS_INFO("Planning is successful");
//  } else
//  {
//    ROS_INFO("Planning is failed");
//  }
//  visual_tools.prompt("next step");


  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot */
//  move_group.move();

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.

  robot_state::RobotStatePtr test_state(new robot_state::RobotState(robot_model));
  psm1_jt_pos.resize(0);
  test_state->copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);
  ROS_INFO("test state joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  robot_state::RobotState start_state(*move_group.getCurrentState());
  psm1_jt_pos.resize(0);
  start_state.copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);
  ROS_INFO("start_state joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }
//  geometry_msgs::Pose start_pose2;
//  start_pose2.orientation.w = 1.0;
//  start_pose2.position.x = 0.55;
//  start_pose2.position.y = -0.05;
//  start_pose2.position.z = 0.8;
//  start_state.setFromIK(psm1_jt_group, start_pose2);

  std::vector< double > position;
  position.push_back(0.0185);
  start_state.setJointPositions("PSM1_outer_insertion", position);
  psm1_jt_pos.resize(0);
  start_state.copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);
  ROS_INFO("start_state joint values after the change of PSM1/outer_insertion: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  move_group.setStartState(start_state);

  daVinciRobotState = move_group.getCurrentState();
  psm1_jt_pos.resize(0);
  daVinciRobotState->copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);

  ROS_INFO("before planning daVinciRobotState joint values: ");
  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the lan.
  psm1_jt_pos[2] = 0.02;  // radians
  move_group.setJointValueTarget(psm1_jt_pos);
  success = (bool) move_group.plan(my_plan);

  daVinciRobotState = move_group.getCurrentState();
  psm1_jt_pos.resize(0);
  daVinciRobotState->copyJointGroupPositions(psm1_jt_group, psm1_jt_pos);
  ROS_INFO("after planning daVinciRobotState joint values: ");

  for(int i = 0; i < psm1_jt_pos.size(); i++)
  {
    ROS_INFO("%dth joint value: %f \n", i, psm1_jt_pos[i]);
  }

  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, psm1_jt_group);
//  visual_tools.trigger();
//  error_code = move_group.move();
//  if (error_code.val == 1)
//  {
//    ROS_INFO("Planning is successful");
//  } else
//  {
//    ROS_INFO("Planning is failed");
//  }
//  visual_tools.prompt("next step");


  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.

  Eigen::Matrix3d rotate_mat(target_pose_1.linear());
  Eigen::Quaterniond q1(rotate_mat);

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "PSM1/tool_tip_link";
  ocm.header.frame_id = "world";
  ocm.orientation.w = q1.w();
  ocm.orientation.x = q1.x();
  ocm.orientation.y = q1.y();
  ocm.orientation.z = q1.z();
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);

  // We will reuse the old goal that we had and plan to it.
  // Note that this will only work if the current state already
  // satisfies the path constraints. So, we need to set the start
  // state to a new pose.

  translation << -0.01, 0.01, 0.05;
  Eigen::Affine3d target_pose_2;

  daVinciRobotState = move_group.getCurrentState();

//  robot_state::RobotState start_state(*move_group.getCurrentState());

  defineToolTipGoal(daVinciRobotState, orientation, translation, target_pose_2);

//  daVinciRobotState->setFromIK(psm1_jt_group, target_pose_2);

//  move_group.setStartState(*daVinciRobotState);

  move_group.setStartStateToCurrentState();
  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  move_group.setPoseTarget(target_pose_2);

  // Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  // Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  move_group.setPlanningTime(10.0);

  success = (bool) move_group.plan(my_plan);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  ROS_INFO_STREAM("target_pose_2_wrt_planning_frame \n" << target_pose_2.matrix());

  target_pose_1= daVinciRobotState->getFrameTransform("/PSM1/tool_tip_link");
  ROS_INFO_STREAM("target_pose_1_wrt_planning_frame \n" << target_pose_1.matrix());


  // Visualize the plan in Rviz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishAxisLabeled(target_pose_2, "goal");
//  visual_tools.publishAxisLabeled(target_pose_1, "start");
//  visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, psm1_jt_group);
//  visual_tools.trigger();
//  error_code = move_group.move();
//  if (error_code.val == 1)
//  {
//    ROS_INFO("Planning is successful");
//  } else
//  {
//    ROS_INFO("Planning is failed");
//  }
//  visual_tools.prompt("next step");


  move_group.clearPathConstraints();
//  error_code = move_group.move();
//  if (error_code.val == 1)
//  {
//    ROS_INFO("Planning is successful");
//  } else
//  {
//    ROS_INFO("Planning is failed");
//  }

  // Cartesian Paths
  // ^^^^^^^^^^^^^^
  // You can plan a cartesian path directly by specifying a list of waypoints
  // for the end-effector to go through. Note that we are starting
  // from the new start state above.  The initial pose (start state) does not
  // need to be added to the waypoint list but adding it can help with visualizations
  geometry_msgs::Pose start_pose2;
  tf::poseEigenToMsg(target_pose_2, start_pose2);

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(start_pose2);

  translation << 0.01, 0.01, 0.05;
  Eigen::Affine3d goal_wrt_current;
  goal_wrt_current.linear() = orientation;
  goal_wrt_current.translation() = translation;
  Eigen::Affine3d target_pose_3;
//  daVinciRobotState = move_group.getCurrentState();
//  defineToolTipGoal(daVinciRobotState, orientation, translation, target_pose_3);

  target_pose_3 = target_pose_1 * goal_wrt_current;
  geometry_msgs::Pose target_pose3;
  tf::poseEigenToMsg(target_pose_3, target_pose3);
  waypoints.push_back(target_pose3);

  translation << 0.01, -0.01, 0.05;
  goal_wrt_current.translation() = translation;
  Eigen::Affine3d target_pose_4;
  target_pose_4 = target_pose_1 * goal_wrt_current;
//  daVinciRobotState = move_group.getCurrentState();
//  defineToolTipGoal(daVinciRobotState, orientation, translation, target_pose_4);
  geometry_msgs::Pose target_pose4;
  tf::poseEigenToMsg(target_pose_4, target_pose4);
  waypoints.push_back(target_pose4);

  translation << -0.01, -0.01, 0.05;
  goal_wrt_current.translation() = translation;
  Eigen::Affine3d target_pose_5;
  target_pose_5 = target_pose_1 * goal_wrt_current;
//  daVinciRobotState = move_group.getCurrentState();
//  defineToolTipGoal(daVinciRobotState, orientation, translation, target_pose_5);
  geometry_msgs::Pose target_pose5;
  tf::poseEigenToMsg(target_pose_5, target_pose5);
  waypoints.push_back(target_pose5);

  ROS_INFO_STREAM("target_pose_2_wrt_planning_frame \n" << target_pose_2.matrix());

  ROS_INFO_STREAM("target_pose_3_wrt_planning_frame \n" << target_pose_3.matrix());

  ROS_INFO_STREAM("target_pose_4_wrt_planning_frame \n" << target_pose_4.matrix());

  ROS_INFO_STREAM("target_pose_5_wrt_planning_frame \n" << target_pose_5.matrix());

  // Cartesian motions are frequently needed to be slower for actions such as approach and retreat
  // grasp motions. Here we demonstrate how to reduce the speed of the robot arm via a scaling factor
  // of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group.setMaxVelocityScalingFactor(0.1);
//  move_group.setStartStateToCurrentState();
  // We want the cartesian path to be interpolated at a resolution of 1 mm
  // which is why we will specify 0.001 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively disabling it.
  // Warning - disabling the jump threshold while operating real hardware can cause
  // large unpredictable motions of redundant joints and could be a safety issue
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.001;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

//  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% acheived)", fraction * 100.0);
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
//  for (std::size_t i = 0; i < waypoints.size(); ++i)
//  {
//    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
//  }
//  visual_tools.trigger();
//  error_code = move_group.move();
//  if (error_code.val == 1)
//  {
//    ROS_INFO("Planning is successful");
//  } else
//  {
//    ROS_INFO("Planning is failed");
//  }
//  visual_tools.prompt("next step");



//  error_code = move_group.move();
//  if (error_code.val == 1)
//  {
//    ROS_INFO("Planning is successful");
//  } else
//  {
//    ROS_INFO("Planning is failed");
//  }
//  // Adding/Removing Objects and Attaching/Detaching Objects
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//  //
//  // Define a collision object ROS message.
//  moveit_msgs::CollisionObject collision_object;
//  collision_object.header.frame_id = move_group.getPlanningFrame();
//
//  // The id of the object is used to identify it.
//  collision_object.id = "box1";
//
//  // Define a box to add to the world.
//  shape_msgs::SolidPrimitive primitive;
//  primitive.type = primitive.BOX;
//  primitive.dimensions.resize(3);
//  primitive.dimensions[0] = 0.005;
//  primitive.dimensions[1] = 0.005;
//  primitive.dimensions[2] = 0.005;
//
//  // Define a pose for the box (specified relative to frame_id)
////    Eigen::Affine3d box_pose;
////
////    Eigen::Matrix3d box_orientation;  // orientation part
////    box_orientation << 1.0, 0.0, 0.0,
////            0.0, 1.0, 0.0,
////            0.0, 0.0, 1.0;
////
////    Eigen::Vector3d box_translation;  // translation part
////    translation << 0.0, 0.5, 0.0;
////
////    box_pose.linear() = box_orientation;
////
////    box_pose.translation() = box_translation;
////
//////    Eigen::Affine3d sca_shaft_link_wrt_world_frame = daVinciRobotState->getFrameTransform("/psm_one_tool_wrist_sca_shaft_link");
////
////    Eigen::Affine3d box_pose_wrt_world_eigen = sca_shaft_link_wrt_world_frame * box_pose;
////
////    geometry_msgs::Pose box_pose_wrt_world;
////    Eigen::Quaternion<double> pose_q(box_pose_wrt_world_eigen.rotation());
////
////    box_pose_wrt_world.orientation.w = pose_q.w();
////    box_pose_wrt_world.orientation.x = pose_q.x();
////    box_pose_wrt_world.orientation.y = pose_q.y();
////    box_pose_wrt_world.orientation.z = pose_q.z();
////
////    box_pose_wrt_world.position.x = box_pose_wrt_world_eigen.translation().x();
////    box_pose_wrt_world.position.y = box_pose_wrt_world_eigen.translation().y();
////    box_pose_wrt_world.position.z = box_pose_wrt_world_eigen.translation().z();
//
//  //Define a pose for the box (specified relative to frame_id)
//  geometry_msgs::Pose box_pose_wrt_world;
//  box_pose_wrt_world.orientation.w = 1.0;
//  box_pose_wrt_world.position.x = -0.25;
//  box_pose_wrt_world.position.y = 0.0;
//  box_pose_wrt_world.position.z = 0.48;
//
//  collision_object.primitives.push_back(primitive);
//  collision_object.primitive_poses.push_back(box_pose_wrt_world);
//  collision_object.operation = collision_object.ADD;
//
//  std::vector<moveit_msgs::CollisionObject> collision_objects;
//  collision_objects.push_back(collision_object);
//
//  // Now, let's add the collision object into the world
//  ROS_INFO_NAMED("tutorial", "Add an object into the world");
//  planning_scene_interface.addCollisionObjects(collision_objects);
//
//  // Show text in Rviz of status
//  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//  visual_tools.trigger();
//
//  // Sleep to allow MoveGroup to recieve and process the collision object message
//  ros::Duration(1.0).sleep();
//  // Now when we plan a trajectory it will avoid the obstacle
//  move_group.setStartState(*move_group.getCurrentState());
//
//
//  orientation << 1.0, 0.0, 0.0,
//    0.0, 1.0, 0.0,
//    0.0, 0.0, 1.0;
//
//  translation << 0.02, 0.0, 0.05;
//
//  psm_1_tool_tip_goal_pose.linear() = orientation;
//
//  psm_1_tool_tip_goal_pose.translation() = translation;
//
//  psm_1_base_wrt_world = daVinciRobotState->getFrameTransform("/PSM1psm_base_link");
//
//  psm_1_tool_tip_wrt_world_frame = daVinciRobotState->getFrameTransform("/PSM1tool_tip_link");
//
//  psm_1_tool_tip_goal_pose_wrt_world = psm_1_tool_tip_wrt_world_frame * psm_1_tool_tip_goal_pose;
//
//  psm_1_tool_tip_goal_pose_wrt_base = psm_1_base_wrt_world.inverse() * psm_1_tool_tip_goal_pose_wrt_world;
//
//  ROS_INFO_STREAM("\n" << psm_1_tool_tip_goal_pose_wrt_base.matrix());
//
//
//  move_group.setPoseTarget(psm_1_tool_tip_goal_pose_wrt_base);
//
//  success = (bool) move_group.plan(my_plan);
//  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");
//
//  // Visualize the plan in Rviz
//  visual_tools.deleteAllMarkers();
//  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
//  visual_tools.publishTrajectoryLine(my_plan.trajectory_, psm1_jt_group);
//  visual_tools.trigger();
//  visual_tools.prompt("next step");
//  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
