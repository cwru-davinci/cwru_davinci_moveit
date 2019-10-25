/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Case Western Reserve University
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
 * Description: A c++ script to do hybrid planner data visualization
 */

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <boost/graph/astar_search.hpp>
#include <iostream>

#include <ros/package.h>
#include "matplotlibcpp.h"
namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace dual_arm_manipulation_planner_interface;

void readPlannerDataStoredToGraphViz()
{
  std::cout << std::endl;
  std::cout << "Reading PlannerData" << std::endl;

  // Recreating the space information from the stored planner data instance
  std::vector <cwru_davinci_grasp::GraspInfo> grasp_poses;
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size()-1, grasp_poses));
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ob::PlannerDataStorage dataStorage;
  ob::PlannerData data(si);

  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  dataStorage.load((dataPath + "/../../../" + "HybridRRTPlannerData").c_str(), data);

  std::ofstream graphVizOutput(dataPath + "/../../../" + "HybridRRTPlannerData.dot");
  data.printGraphviz(graphVizOutput);
  graphVizOutput.close();
}

void readPlannerData()
{
  std::cout << std::endl;
  std::cout << "Reading PlannerData" << std::endl;

  // Recreating the space information from the stored planner data instance
  std::vector <cwru_davinci_grasp::GraspInfo> grasp_poses;
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size()-1, grasp_poses));
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ob::PlannerDataStorage dataStorage;
  ob::PlannerData data(si);

  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  dataStorage.load((dataPath + "/../../../" + "HybridRRTPlannerData").c_str(), data);
  
  // Re-extract the shortest path from the loaded planner data
  bool isStart = data.isStartVertex(0);
  bool isGoal = data.isGoalVertex(data.numVertices() - 1);
  
}

int main(int argc, char** argv)
{
  // readPlannerDataStoredToGraphViz();
  readPlannerData();
  return 0;
}
