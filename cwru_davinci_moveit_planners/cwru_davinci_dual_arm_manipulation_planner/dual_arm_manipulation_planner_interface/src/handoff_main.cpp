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
 * Description: handoff main function
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>

#include <dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <dual_arm_manipulation_planner_interface//hybrid_motion_validator.h>

// Grasp generation and visualization
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>

// c++
#include <fstream>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace dual_arm_manipulation_planner_interface;

void plan
(
const ros::NodeHandle& node_handle,
const ros::NodeHandle& node_handle_priv,
std::vector<cwru_davinci_grasp::GraspInfo> grasp_poses
)
{
  std::string objectName = "needle_r";
  robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
  // create an instance of state space
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size() - 1, grasp_poses));

  // construct an instance of space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  double se3Bounds[6];
  if (!node_handle_priv.hasParam("se3_bounds"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `se3_bounds` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << node_handle_priv.getNamespace());
    return;
  }

  XmlRpc::XmlRpcValue xmlSE3BoundsArray;
  node_handle_priv.getParam("se3_bounds", xmlSE3BoundsArray);
  if (xmlSE3BoundsArray.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (std::size_t i = 0; i < xmlSE3BoundsArray.size(); ++i)
    {
      ROS_ASSERT(xmlSE3BoundsArray[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      se3Bounds[i] = static_cast<double>(xmlSE3BoundsArray[i]);
    }
  }
  else
  {
    ROS_ERROR_STREAM("SE3 bounds type is not type array?");
  }

  hystsp->setSE3Bounds(se3Bounds[0],
                       se3Bounds[1],
                       se3Bounds[2],
                       se3Bounds[3],
                       se3Bounds[4],
                       se3Bounds[5]);

  si->setStateValidityChecker(
  std::make_shared<HybridStateValidityChecker>(si, robotModelLoader.getModel(), objectName));
  si->setMotionValidator(
  std::make_shared<HybridMotionValidator>(si, robotModelLoader.getModel(), objectName));

  si->setup();

  ob::StateSamplerPtr stateSampler = si->allocStateSampler();  // assign HybridStateSampler to stateSampler

  // create a random start state
  ob::ScopedState<HybridObjectStateSpace> start(hystsp);
  ob::ScopedState<HybridObjectStateSpace> goal(hystsp);

  bool is_ss_valid = false;
  while (!is_ss_valid)
  {
    stateSampler->sampleUniform(start.get());
    is_ss_valid = si->isValid(start.get());
  }

//
//  stateSampler->sampleUniform(goal.get());
//
//  int start_arm_index = start->armIndex().value;
//  int start_grasp_part = grasp_poses[start->graspIndex().value].part_id;
//  int goal_arm_index = goal->armIndex().value;
//  int goal_grasp_part = grasp_poses[goal->graspIndex().value].part_id;
//  bool same_arm = (start_arm_index == goal_arm_index) ? true : false;
//  bool same_grasp_part = (start_grasp_part == goal_grasp_part) ? true : false;

//  bool is_gs_valid = si->isValid(goal.get());
//  while (!same_arm || same_grasp_part || !is_gs_valid)
//  {
//    stateSampler->sampleUniform(goal.get());
//    is_gs_valid = si->isValid(goal.get());
//    if (!is_gs_valid)
//      continue;
//    same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
//    same_grasp_part = (start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
//  }

  bool is_gs_valid = false;
  while (!is_gs_valid)
  {
    stateSampler->sampleUniform(goal.get());
    goal->graspIndex().value = 147;
    goal->armIndex().value = 1;
    goal->setJointsComputed(false);
    is_gs_valid = si->isValid(goal.get());
  }

  std::string packPath;
  node_handle_priv.getParam("packPath", packPath);

  {
    std::ofstream outFile(packPath + "/../../../" + "SampledGoalState.txt");
    outFile << std::setprecision(15) << goal->se3State().getX() << ", "
                                     << goal->se3State().getY() << ", "
                                     << goal->se3State().getZ() << ", "
                                     << goal->se3State().rotation().x << ", "
                                     << goal->se3State().rotation().y << ", "
                                     << goal->se3State().rotation().z << ", "
                                     << goal->se3State().rotation().w << ",\n";
    outFile << std::setprecision(15) << goal->jointVariables()[0] << ", "
                                     << goal->jointVariables()[1] << ", "
                                     << goal->jointVariables()[2] << ", "
                                     << goal->jointVariables()[3] << ", "
                                     << goal->jointVariables()[4] << ", "
                                     << goal->jointVariables()[5] << ",\n";
    outFile.close();
  }

  // create a problem instance
  auto pdef(std::make_shared<ob::ProblemDefinition>(si));

  // set the start and goal states
  pdef->setStartAndGoalStates(start, goal);

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

  if (solved)
  {
    if (pdef->hasExactSolution())
    {
      std::cout << "Has exact solution" << std::endl;
      double* total_time = new double;
      si->getStateSpace().get()->as<HybridObjectStateSpace>()->printExecutionDuration(total_time);
      delete total_time;

      og::PathGeometric slnPath = *(pdef->getSolutionPath()->as<og::PathGeometric>());
      // print the path to screen
      std::cout << "Found solution:\n" << "\n";
      std::cout << "Found solution with " << slnPath.getStateCount()
                << " states and length " << slnPath.length() << std::endl;
      // print the path to screen

      std::ofstream outFile(packPath + "/../../../" + "PathFound.txt");
      slnPath.printAsMatrix(outFile);
      outFile.close();
    }
    else
    {
      std::cout << "Do not have exact solution" << std::endl;
    }
  }
  else
  {
    std::cout << "No solution found" << std::endl;
  }
  std::cout << "Writing PlannerData to file’ HybridRRTPlannerData’" << std::endl;
  ob::PlannerData data(si);
  planner->getPlannerData(data);
  data.computeEdgeWeights();

  std::cout << "Found " << data.numVertices() << " vertices " << "\n";
  std::cout << "Found " << data.numEdges() << " edges " << "\n";
  std::cout << "Actual Planning Time is: " << planning_time.count() << std::endl;

  ob::PlannerDataStorage dataStorage;
  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  dataStorage.store(data, (dataPath + "/../../../" + "HybridRRTPlannerData").c_str());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "handoff_main");

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");
  ros::Duration(3.0).sleep();

  cwru_davinci_grasp::DavinciNeedleGrasperBasePtr simpleGrasp =
  boost::make_shared<cwru_davinci_grasp::DavinciNeedleGrasperBase>(node_handle_priv,
                                                                   "psm_one",
                                                                   "psm_one_gripper");

  std::vector<cwru_davinci_grasp::GraspInfo> grasp_poses = simpleGrasp->getAllPossibleNeedleGrasps(false);

  plan(node_handle, node_handle_priv, grasp_poses);

  return 0;
}
