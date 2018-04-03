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

#include <ros/ros.h>
#include <string>
#include <vector>

#include <cwru_davinci_moveit_object_handling/davinci_grasped_object_handler.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <eigen_conversions/eigen_msg.h>


int main(int argc, char **argv)
{

  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << " <object-name> [--detach]" << std::endl;
    return 1;
  }

  std::string obj_name = argv[1];

  bool is_detach = false;
  if (argc > 2)
  {
    std::string arg = argv[2];
    if ( arg == "--detach");
    {
      ROS_INFO("detach needle");
      is_detach = true;
    }
  }

  ros::init(argc, argv, "davinci_needle_attachment_moveit_test");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
//  ros::Duration(4.0).sleep();
  spinner.start();


  const std::string PLANNING_GROUP = "psm_one";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//  const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  const std::string gripper_name = move_group.getEndEffector();

  moveit::planning_interface::MoveGroupInterface gripper_group(gripper_name);
  const std::vector<std::string> gripper_link_names = gripper_group.getLinkNames();

  for (int i = 0; i < gripper_link_names.size(); i++)
  {
    ROS_INFO("the %dth gripper link is %s", i, gripper_link_names[i].c_str());
  }


  std::string get_planning_scene("get_planning_scene");
  std::string set_planning_scene("planning_scene");

  davinci_moveit_object_handling::DavinciMoveitGraspedObjectHandler graspedObjectHandler(node_handle,
                                                                                         gripper_link_names,
                                                                                         get_planning_scene,
                                                                                         set_planning_scene);

  graspedObjectHandler.waitForSubscribers();

//  static const std::string PLANNING_GROUP = "psm_one";
//  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//  const robot_state::JointModelGroup *joint_model_group =
//    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
//
//  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
//  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
//  ROS_INFO_NAMED("tutorial", "End effector name: %s", move_group.getEndEffector().c_str());
//
//  moveit_msgs::CollisionObject needle;
//  needle.header.frame_id = move_group.getPlanningFrame();
//
//  collision_object.id = "needle_r";


  if(!is_detach)
  {
    graspedObjectHandler.attachObjectToRobot(obj_name, move_group.getEndEffectorLink());
  }
  else
  {
    graspedObjectHandler.detachObjectFromRobot(obj_name);
  }

  ros::Duration(1).sleep();
  return 0;
}
