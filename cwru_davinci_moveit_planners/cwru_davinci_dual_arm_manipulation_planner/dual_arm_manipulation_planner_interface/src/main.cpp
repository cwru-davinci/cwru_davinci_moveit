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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <moveit/dual_arm_manipulation_planner_interface//hybrid_motion_validator.h>
#include <moveit/dual_arm_manipulation_planner_interface/hybrid_valid_state_sampler.h>
#include <moveit/dual_arm_manipulation_planner_interface/davinci_needle_handoff_execution.h>

//moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Grasp generation and visualization
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace dual_arm_manipulation_planner_interface;

// return an sampler
ob::ValidStateSamplerPtr allocHybridValidStateSampler(const ob::SpaceInformation *si)
{

  // we can perform any additional setup / configuration of a sampler here,
  // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
  return std::make_shared<HybridValidStateSampler>("robot_description", si);
}

void plan(const ros::NodeHandle &node_handle,
          const ros::NodeHandle &node_handle_priv,
          const std::vector<double> ss_needle_pose_translation,
          const std::vector<double> ss_needle_pose_orientation,
          const std::vector<double> gs_needle_pose_translation,
          const std::vector<double> gs_needle_pose_orientation,
          const std::vector<double> ss_joint_values,
          const std::vector<double> gs_joint_values,
          int ss_arm_index,
          int gs_arm_index,
          int ss_grasp_pose_index,
          int gs_grasp_pose_index,
          std::vector<cwru_davinci_grasp::GraspInfo> grasp_poses)
{

//  for(int i = 0; i < grasp_poses.size(); i++)
//  {
//    if(grasp_poses[i].graspParamInfo.grasp_id == 121)
//    {
//      ss_grasp_pose_index = i;
//      continue;
//    }
//    else if(grasp_poses[i].graspParamInfo.grasp_id == 121)
//    {
//      gs_grasp_pose_index = i;
//      continue;
//    }
//    else
//    {
//      if (grasp_poses[ss_grasp_pose_index].graspParamInfo.grasp_id ==
//          121 && grasp_poses[gs_grasp_pose_index].graspParamInfo.grasp_id == 121)
//      {
//        break;
//      }
//    }
//  }
//  gs_grasp_pose_index = ss_grasp_pose_index;

  std::string object_name = "needle_r";
  std::string robot_name = "robot_description";
  // create an instance of state space
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size(), grasp_poses));

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ompl::base::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.2);
  se3_xyz_bounds.setHigh(0, 0.2);
  se3_xyz_bounds.setLow(1, -0.06);
  se3_xyz_bounds.setHigh(1, 0.09);
  se3_xyz_bounds.setLow(2, 0.266);
  se3_xyz_bounds.setHigh(2, 0.496);

  hystsp->setSE3Bounds(se3_xyz_bounds);


  si->setStateValidityChecker(
    std::make_shared<HybridStateValidityChecker>(node_handle, robot_name, object_name, si));
// or this call:
//  si->setStateValidityCheckingResolution(0.03); // 3%
  si->setMotionValidator(
    std::make_shared<HybridMotionValidator>(node_handle, node_handle_priv, robot_name, object_name, si));

  si->setup();

  // create a random start state
  ob::ScopedState<HybridObjectStateSpace> start(hystsp);
  ob::ScopedState<HybridObjectStateSpace> goal(hystsp);

  start->se3State().setXYZ(ss_needle_pose_translation[0],
                           ss_needle_pose_translation[1],
                           ss_needle_pose_translation[2]);
  start->se3State().rotation().setAxisAngle(ss_needle_pose_orientation[0],
                                            ss_needle_pose_orientation[1],
                                            ss_needle_pose_orientation[2],
                                            ss_needle_pose_orientation[3]);
  start->armIndex().value = ss_arm_index;  // set arm index
//  for(int i = 0; i < grasp_poses.size(); i++)
//  {
    start->graspIndex().value = 0;
//    if(!si->isValid(start.get()))
//      ros::shutdown();
//      std::cout << "Initial state selected Grasp's part is " << grasp_poses[start->graspIndex().value].part_id << std::endl;
//  }
//  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
//
//  moveit::core::RobotStatePtr daVinciRobotState(new robot_state::RobotState(robot_model));
//  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
//
//  static const std::string PSM_ONE_PLANNING_GROUP = "psm_one";
//  static const std::string PSM_TWO_PLANNING_GROUP = "psm_two";
//
//  const robot_state::JointModelGroup* psm1_jt_group_parent = daVinciRobotState->getJointModelGroup(PSM_ONE_PLANNING_GROUP);
//  daVinciRobotState->setToDefaultValues(psm1_jt_group_parent, "psm_one_home");
//  const moveit::core::LinkModel* tip_link = psm1_jt_group_parent->getOnlyOneEndEffectorTip();
//  Eigen::Affine3d tip_frame_wrt_world = daVinciRobotState->getGlobalLinkTransform(tip_link);
//
//  Eigen::Affine3d needle_wrt_world = tip_frame_wrt_world * grasp_poses[gs_grasp_pose_index].grasp_pose;
//  Eigen::AngleAxisd angle_ax(needle_wrt_world.linear());
//  Eigen::Vector3d axis = angle_ax.axis();
//  axis.normalize();
//
//  goal->se3State().setXYZ(needle_wrt_world.translation()[0],
//                          needle_wrt_world.translation()[1],
//                          needle_wrt_world.translation()[2]);
//  goal->se3State().rotation().setAxisAngle(axis(0),
//                                           axis(1),
//                                           axis(2),
//                                           angle_ax.angle());

  goal->se3State().setXYZ(gs_needle_pose_translation[0],
                          gs_needle_pose_translation[1],
                          gs_needle_pose_translation[2]);
  goal->se3State().rotation().setAxisAngle(gs_needle_pose_orientation[0],
                                           gs_needle_pose_orientation[1],
                                           gs_needle_pose_orientation[2],
                                           gs_needle_pose_orientation[3]);
  goal->armIndex().value = gs_arm_index;  // set arm index
//  goal->graspIndex().value = 0;
  for(int i = 0; i < grasp_poses.size(); i++)
  {
    if(grasp_poses[i].part_id == 0)
    {
      goal->graspIndex().value = i;
      if(si->isValid(goal.get()))
        break;
    }
  }

  std::cout << "Goal state selected Grasp's part is " << grasp_poses[goal->graspIndex().value].part_id << std::endl;
  bool valid_ss = si->isValid(start.get());
  bool valid_gs = si->isValid(goal.get());

  hystsp->printState(start.get(), std::cout);
  hystsp->printState(goal.get(), std::cout);

  double distance_btw_s_g = hystsp->distance(start.get(), goal.get());
  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);

  si->setValidStateSamplerAllocator(allocHybridValidStateSampler);
  // create a planner for the defined space
  auto planner(std::make_shared<og::RRTConnect>(si));
  // set the problem we are trying to solve for the planner
  planner->setProblemDefinition(pdef);
  planner->setRange(100.0);
  // perform setup steps for the planner
  planner->setup();
  // print the settings for this space
  si->printSettings(std::cout);
  // print the problem settings
  pdef->print(std::cout);
  // attempt to solve the problem within one second of planning time
  auto start_ts = std::chrono::high_resolution_clock::now();
  si->getStateSpace().get()->as<HybridObjectStateSpace>()->resetTimer();
  ob::PlannerStatus solved = planner->ob::Planner::solve(500.0);

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> planning_time = finish - start_ts;

//  planner->getPlannerData(plannedData);
  si->getStateSpace().get()->as<HybridObjectStateSpace>()->printExecutionDuration(nullptr, true);
  if (solved)
  {
    if(pdef->hasExactSolution())
    {
      std::cout << "Has exact solution" << std::endl;
    }
    else
    {
      std::cout << "Do not have exact solution" << std::endl;
    }


    og::PathGeometric slnPath = *(pdef->getSolutionPath()->as<og::PathGeometric>());
    // print the path to screen
    std::cout << "Found solution:\n" << std::endl;
    std::cout << "Found solution with " << slnPath.getStateCount()
                 << " states and length " << slnPath.length( ) << std::endl;
    // print the path to screen
    slnPath.printAsMatrix(std::cout);

//    DavinciNeedleHandoffExecution handOffExecute(slnPath, hystsp->getJointSpaceDimension());

    std::cout << "Writing PlannerData to file’./myPlannerData’" << std::endl;
    ob::PlannerData plannedData(si);
    planner->getPlannerData(plannedData);
    plannedData.computeEdgeWeights();

    std::cout << "Found " << plannedData.numVertices() << " vertices " << std::endl;
    std::cout << "Found " << plannedData.numEdges ( ) << " edges " << std::endl;
    std::cout << "Actual Planning Time is: " << planning_time.count() << "s\n";
//
//    for(int i = 0; i < plannedData.numVertices(); i++)
//    {
//      plannedData.getVertex(i).getState()->as<HybridObjectStateSpace::StateType>()->clearJointValues();
//    }
//
//    plannedData.printGraphviz();
//    plannedData.printGraphML();
//    ob::PlannerDataStorage dataStorage;
//    dataStorage.store(plannedData , "myPlannerData");
  }
  else
    std::cout << "No solution found" << std::endl;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "handoff_main");
  ros::AsyncSpinner spinner(1);
  ros::Duration(3.0).sleep();
  spinner.start();

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");
  std::string object_name = "needle_r";
  std::string robot_name = "robot_description";

  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr simpleGrasp =
    boost::make_shared<cwru_davinci_grasp::DavinciSimpleNeedleGrasper>(
      node_handle, node_handle_priv, "psm_one", object_name);

  std::vector<double> ss_needle_pose_translation;
  std::vector<double> ss_needle_pose_orientation;
  std::vector<double> gs_needle_pose_translation;
  std::vector<double> gs_needle_pose_orientation;
  std::vector<double> ss_joint_values;
  std::vector<double> gs_joint_values;
  int ss_arm_index;
  int ss_grasp_pose_index;
  int gs_arm_index;
  int gs_grasp_pose_index;

  if (node_handle_priv.hasParam("ss_needle_pose_translation"))
  {
    XmlRpc::XmlRpcValue needle_pose_list;
    node_handle_priv.getParam("ss_needle_pose_translation", needle_pose_list);

    ROS_ASSERT(needle_pose_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_pose_list.size(); ++i)
    {
      ROS_ASSERT(needle_pose_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      ss_needle_pose_translation.push_back(static_cast<double>(needle_pose_list[i]));
    }
  }

  if (node_handle_priv.hasParam("ss_needle_pose_orientation"))
  {
    XmlRpc::XmlRpcValue needle_ori_list;
    node_handle_priv.getParam("ss_needle_pose_orientation", needle_ori_list);

    ROS_ASSERT(needle_ori_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_ori_list.size(); ++i)
    {
      ROS_ASSERT(needle_ori_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      ss_needle_pose_orientation.push_back(static_cast<double>(needle_ori_list[i]));
    }
  }

  if (node_handle_priv.hasParam("gs_needle_pose_translation"))
  {
    XmlRpc::XmlRpcValue needle_pose_list;
    node_handle_priv.getParam("gs_needle_pose_translation", needle_pose_list);

    ROS_ASSERT(needle_pose_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_pose_list.size(); ++i)
    {
      ROS_ASSERT(needle_pose_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      gs_needle_pose_translation.push_back(static_cast<double>(needle_pose_list[i]));
    }
  }

  if (node_handle_priv.hasParam("gs_needle_pose_orientation"))
  {
    XmlRpc::XmlRpcValue needle_ori_list;
    node_handle_priv.getParam("gs_needle_pose_orientation", needle_ori_list);

    ROS_ASSERT(needle_ori_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_ori_list.size(); ++i)
    {
      ROS_ASSERT(needle_ori_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      gs_needle_pose_orientation.push_back(static_cast<double>(needle_ori_list[i]));
    }
  }

  if (node_handle_priv.hasParam("ss_joint_values"))
  {
    XmlRpc::XmlRpcValue joint_value_list;
    node_handle_priv.getParam("ss_joint_values", joint_value_list);

    ROS_ASSERT(joint_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < joint_value_list.size(); ++i)
    {
      ROS_ASSERT(joint_value_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      ss_joint_values.push_back(static_cast<double>(joint_value_list[i]));
    }
  }

  if (node_handle_priv.hasParam("gs_joint_values"))
  {
    XmlRpc::XmlRpcValue joint_value_list;
    node_handle_priv.getParam("gs_joint_values", joint_value_list);

    ROS_ASSERT(joint_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < joint_value_list.size(); ++i)
    {
      ROS_ASSERT(joint_value_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      gs_joint_values.push_back(static_cast<double>(joint_value_list[i]));
    }
  }

  node_handle_priv.getParam("ss_arm_index", ss_arm_index);
  node_handle_priv.getParam("ss_grasp_pose_index", ss_grasp_pose_index);
  node_handle_priv.getParam("gs_arm_index", gs_arm_index);
  node_handle_priv.getParam("gs_grasp_pose_index", gs_grasp_pose_index);

  std::vector <cwru_davinci_grasp::GraspInfo> grasp_poses = simpleGrasp->getAllPossibleNeedleGrasps(true);

  plan(node_handle, node_handle_priv,
       ss_needle_pose_translation,
       ss_needle_pose_orientation,
       gs_needle_pose_translation,
       gs_needle_pose_orientation,
       ss_joint_values,
       gs_joint_values,
       ss_arm_index,
       gs_arm_index,
       ss_grasp_pose_index,
       gs_grasp_pose_index, grasp_poses);

  return 0;
}
