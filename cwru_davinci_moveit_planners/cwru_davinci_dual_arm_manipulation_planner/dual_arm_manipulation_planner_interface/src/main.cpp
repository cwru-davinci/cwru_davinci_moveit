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
 * Description:
 */


//class HybridNode

//{
//public:
//  HybridNode(const ompl::base::state& state, HybridNode* parent = nullptr)
//  : parent_(parent), state_(state)
//  {
//    if (_parent)
//    {
//      _parent->_children.push_back(this);
//    }
//  }
//
//  const HybridNode* parent()
//  {
//    return parent_;
//  }
//
//private:
//  HybridNode parent_;
//
//  ompl::state::base* state_;
//
//  std::vector<HybridNode> children_;
//};
//
//class HybridTree
//{
//public:
//  HybridTree(const )
//private:
//
//  HybridNode h_node_;
//
//
//
//};
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>

//moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Grasp generation and visualization
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace dual_arm_manipulation_planner_interface;

bool isStateValid(const ob::State *state)
{
  // cast the abstract state type to the type we expect
  const auto *hybrid_state = state->as<HybridObjectStateSpace::StateType>();

  // extract the first component of the state and cast it to what we expect
  const auto *object_se3state = hybrid_state->as<ob::SE3StateSpace::StateType>(0);

  const auto *object_pos = object_se3state->as<ob::RealVectorStateSpace::StateType>(0);
  const auto *object_rot = object_se3state->as<ob::SO3StateSpace::StateType>(1);

  // extract the second component of the state and cast it to what we expect
  const auto *arm_index = hybrid_state->as<ob::DiscreteStateSpace::StateType>(1);

  // extract the third component of the state and cast it to what we expect
  const auto *grasp_index = hybrid_state->as<ob::DiscreteStateSpace::StateType>(2);



  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
//  return (const void*)rot != (const void*)pos;
}

void plan(const geometry_msgs::PoseStamped &object_init_pose, const geometry_msgs::PoseStamped &object_goal_pose,
          const int &initial_arm_index, const int &initial_grasp_id, const int &goal_arm_index,
          const int &goal_grasp_id)
{

  // construct the state space we are planning in
  auto space(std::make_shared<HybridObjectStateSpace>(1, 2, 0, 100));

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(space));

  // set state validity checking for this space
  si->setStateValidityChecker(isStateValid);


  // create a random start state
  ob::ScopedState<HybridObjectStateSpace> start(space);

  start->as<ob::SE3StateSpace::StateType>(0)->setXYZ(object_init_pose.pose.position.x,
                                                     object_init_pose.pose.position.y,
                                                     object_init_pose.pose.position.z);

  start->as<ob::SE3StateSpace::StateType>(0)->rotation().x = object_init_pose.pose.orientation.x;
  start->as<ob::SE3StateSpace::StateType>(0)->rotation().y = object_init_pose.pose.orientation.y;
  start->as<ob::SE3StateSpace::StateType>(0)->rotation().z = object_init_pose.pose.orientation.z;
  start->as<ob::SE3StateSpace::StateType>(0)->rotation().w = object_init_pose.pose.orientation.w;

  start->as<ob::DiscreteStateSpace::StateType>(1)->value = initial_arm_index;  // set arm index
  start->as<ob::DiscreteStateSpace::StateType>(2)->value = initial_grasp_id;  // set grasp index

  // create a random goal state
  ob::ScopedState<HybridObjectStateSpace> goal(space);

  goal->as<ob::SE3StateSpace::StateType>(0)->setXYZ(object_goal_pose.pose.position.x,
                                                    object_goal_pose.pose.position.y,
                                                    object_goal_pose.pose.position.z);

  goal->as<ob::SE3StateSpace::StateType>(0)->rotation().x = object_goal_pose.pose.orientation.x;
  goal->as<ob::SE3StateSpace::StateType>(0)->rotation().y = object_goal_pose.pose.orientation.y;
  goal->as<ob::SE3StateSpace::StateType>(0)->rotation().z = object_goal_pose.pose.orientation.z;
  goal->as<ob::SE3StateSpace::StateType>(0)->rotation().w = object_goal_pose.pose.orientation.w;

  goal->as<ob::DiscreteStateSpace::StateType>(1)->value = goal_arm_index;  // set arm index
  goal->as<ob::DiscreteStateSpace::StateType>(2)->value = goal_grasp_id;  // set grasp index

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);

  // create a planner for the defined space
  auto planner(std::make_shared<og::RRTConnect>(si));

  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);

  // perform setup steps for the planner
  planner->setup();


  // print the settings for this space
  si->printSettings(std::cout);

  // print the problem settings
  pdef->print(std::cout);

  // attempt to solve the problem within one second of planning time
  ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

  if (solved)
  {
    // get the goal representation from the problem definition (not the same as the goal state)
    // and inquire about the found path
    ob::PathPtr path = pdef->getSolutionPath();
    std::cout << "Found solution:" << std::endl;

    // print the path to screen
    path->print(std::cout);
  }
  else
    std::cout << "No solution found" << std::endl;
}


int main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cerr << "Usage: " << argv[0] << "<which_arm> <needle_name> [place]"
              << std::endl;
    return 1;
  }

  std::string which_arm = argv[1];
  std::string needle_name = argv[2];

  bool is_place = false;
  if (argc > 3)
  {
    std::string arg = argv[3];
    if (arg == "place")
    {
      ROS_INFO("Place %s", needle_name.c_str());
      is_place = true;
    }
  }
  else
  {
    ROS_INFO("Pick %s", needle_name.c_str());
  }

  ros::init(argc, argv, "dual_arm_planning_main_node ");
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");

  ros::AsyncSpinner spinner(1);
  ros::Duration(5.0).sleep();
  spinner.start();

  cwru_davinci_grasp::DavinciSimpleNeedleGrasper needleGrasper(node_handle,
                                                               node_handle_priv,
                                                               needle_name,
                                                               which_arm);

  geometry_msgs::Pose needle_pose_goal;

  std::vector<moveit_msgs::Grasp> possible_grasps = needleGrasper.getAllPossibleNeedleGrasps();

  return 0;
}
