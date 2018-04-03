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

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "psm_one_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();


    // BEGIN_TUTORIAL
    //
    // Setup
    // ^^^^^
    //
    // The :planning_scene:`PlanningScene` class can be easily setup and
    // configured using a :moveit_core:`RobotModel` or a URDF and
    // SRDF. This is, however, not the recommended way to instantiate a
    // PlanningScene. The :planning_scene_monitor:`PlanningSceneMonitor`
    // is the recommended method to create and maintain the current
    // planning scene (and is discussed in detail in the next tutorial)
    // using data from the robot's joints and the sensors on the robot. In
    // this tutorial, we will instantiate a PlanningScene class directly,
    // but this method of instantiation is only intended for illustration.

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    // Collision Checking
    // ^^^^^^^^^^^^^^^^^^
    //
    // Self-collision checking
    // ~~~~~~~~~~~~~~~~~~~~~~~
    //
    // The first thing we will do is check whether the robot in its
    // current state is in *self-collision*, i.e. whether the current
    // configuration of the robot would result in the robot's parts
    // hitting each other. To do this, we will construct a
    // :collision_detection_struct:`CollisionRequest` object and a
    // :collision_detection_struct:`CollisionResult` object and pass them
    // into the collision checking function. Note that the result of
    // whether the robot is in self-collision or not is contained within
    // the result. Self collision checking uses an *unpadded* version of
    // the robot, i.e. it directly uses the collision meshes provided in
    // the URDF with no extra padding added on.

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current State is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Change the state
    // ~~~~~~~~~~~~~~~~
    //
    // Now, let's change the current state of the robot. The planning
    // scene maintains the current state internally. We can get a
    // reference to it and change it and then check for collisions for the
    // new robot configuration. Note in particular that we need to clear
    // the collision_result before making a new collision checking
    // request.

    robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 2: Current State is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // Checking for a group
    // ~~~~~~~~~~~~~~~~~~~~
    //
    // Now, we will do collision checking only for the psm_one of the
    // daVinci, i.e. we will check whether there are any collisions between
    // the psm_one and other parts of the body of the robot. We can ask
    // for this specifically by adding the group name "psm_one" to the
    // collision request.

    collision_request.group_name = std::string("psm_one");
    current_state.setToRandomPositions();
    collision_result.clear();
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    // END_TUTORIAL

    ros::shutdown();
    return 0;

}