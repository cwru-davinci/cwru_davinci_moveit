/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


bool isTwoVectorSame(const std::vector<double>& joint_values_fk, const std::vector<double>& joint_values_ik)
{
  double diff = 1e-3;
  bool is_same = false;
  if(joint_values_fk.size() == joint_values_ik.size())
  {
    for (int i = 0; i < joint_values_fk.size(); i++)
    {
      if(joint_values_fk[i] - joint_values_ik[i] >= diff)
      {
        is_same = false;
        return false;
      }
    }
    is_same = true;
    return is_same;
  }
  else
  {
    return is_same;
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "psm_one_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using the RobotModel class is very easy. In
  // general, you will find that most higher-level components will
  // return a shared pointer to the RobotModel. You should always use
  // that when possible. In this example, we will start with such a
  // shared pointer and discuss only the basic API. You can have a
  // look at the actual code API for these classes to get more
  // information about how to use more features provided by these
  // classes.
  //
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

  ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());

//    planning_scene::PlanningScene planning_scene(kinematic_model);
//    ROS_INFO_STREAM("\n" << planning_scene.getFrameTransform("/psm_one_tool_wrist_sca_shaft_link").matrix());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "right_arm" of the PR2
  // robot.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  ROS_INFO_STREAM("\n" << kinematic_state->getFrameTransform("PSM1tool_wrist_sca_shaft_link").matrix());

  ROS_INFO_STREAM("\n" << kinematic_state->getGlobalLinkTransform("PSM1tool_wrist_sca_shaft_link").matrix());

  kinematic_state->setToDefaultValues();

  const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("psm_one");

  const std::vector<std::string> &joint_names = joint_model_group->getVariableNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the right arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the right arm outside its joint limit */
  joint_values[2] = 10.0;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  joint_values.clear();
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  ROS_INFO_STREAM("After updating joint 3");
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "psm_one_tool_wrist_sca_shaft_link" which is the most distal link in the
  // "psm_one" of the robot.

  std::vector<double> joint_values_fk;
  std::vector<double> joint_values_ik;

  int num_test = 50;
  std::vector<int> result_vec;
  for(int i = 0; i < num_test; i++)
  {
    kinematic_state->setToRandomPositions(joint_model_group);

    const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(
      "PSM1tool_wrist_sca_shaft_link");

    /* Print end-effector pose. Remember that this is in the model frame */
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

//    ROS_INFO_STREAM("\n" << planning_scene.getFrameTransform("/psm_one_tool_wrist_sca_shaft_link").matrix());
    ROS_INFO_STREAM("\n" << kinematic_state->getFrameTransform("/PSM1tool_wrist_sca_shaft_link").matrix());
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values_fk);
    // Inverse Kinematics
    // ^^^^^^^^^^^^^^^^^^
    // We can now solve inverse kinematics (IK) for the psm one of the daVinci robot.
    // To solve IK, we will need the following:
    // * The desired pose of the end-effector (by default, this is the last link in the
    // "psm_one_tool_wrist_sca_shaft_link" chain): end_effector_state that we computed in the step above.
    // * The number of attempts to be made at solving IK: 5
    // * The timeout for each attempt: 0.1 s
    bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 1, 0.5);

    // Now, we can print out the IK solution (if found) :
    if(found_ik)
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values_ik);
      if(isTwoVectorSame(joint_values_fk, joint_values_fk))
      {
//        ROS_INFO("The FK joint values is equal to IK solution");
//        for(std::size_t i = 0; i < joint_names.size(); ++i)
//        {
//          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values_ik[i]);
//        }
        result_vec.push_back(1);
      }
      else
      {
//        ROS_INFO("The FK joint values is NOT NOT NOT equal to IK solution");
        result_vec.push_back(2);
      }
    }
    else
    {
//      ROS_INFO("Did not find IK solution");
      result_vec.push_back(0);
    }
  }

  for(int i = 0; i<result_vec.size(); i++)
  {
    switch(result_vec[i])
    {
      case 0 :
      {
        ROS_INFO("NO solution");
        break;
      }

      case 1 :
      {
        ROS_INFO("Has same solution");
        break;
      }
      case 2 :
      {
        ROS_INFO("Has different solution");
        break;
      }

    }
  }

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}