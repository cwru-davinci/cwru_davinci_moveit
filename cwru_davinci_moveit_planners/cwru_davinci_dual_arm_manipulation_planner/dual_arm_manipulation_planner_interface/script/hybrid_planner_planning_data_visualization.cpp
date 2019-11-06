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

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <cwru_davinci_grasp/davinci_needle_grasper_base.h>
#include <dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/astar_search.hpp>
#include <iostream>

#include <ros/package.h>
#include "matplotlibcpp.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace plt = matplotlibcpp;
using namespace dual_arm_manipulation_planner_interface;

class TreeDrawingDfsVisitor : public boost::default_dfs_visitor
{

};

class HyperPlaneDrawing
{
public:
  HyperPlaneDrawing(const double ws_x_min,
                    const double ws_x_max,
                    const double ws_y_min,
                    const double ws_y_max,
                    const double ws_z_min,
                    const double ws_z_max,
                    const int plane_num)
                    : x_min_(ws_x_min),
                      x_max_(ws_x_max),
                      y_min_(ws_y_min),
                      y_max_(ws_y_max),
                      z_min_(ws_z_min),
                      z_max_(ws_z_max),
                      plane_num_(plane_num)
  {}

  void drawSubCube();
protected:
  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double z_min_;
  double z_max_;

  double spacing_;

  int plane_num_;
  std::vector<double> hyper_plane_x;
  std::vector<double> hyper_plane_y;

  std::vector<std::vector<double>> hyper_plane_z;
};

void HyperPlaneDrawing::drawSubCube()
{
  std::vector<std::vector<double>> x {std::vector<double>{x_min_, x_min_, x_min_, x_min_, x_max_, x_max_, x_max_, x_max_}};
  std::vector<std::vector<double>> y {std::vector<double>{y_min_, y_max_, y_max_, y_min_, y_min_, y_max_, y_max_, y_min_}};
  std::vector<std::vector<double>> z {std::vector<double>{z_min_, z_min_, z_max_, z_max_, z_min_, z_min_, z_max_, z_max_}};
  plt::plot_surface(x, y, z);
  plt::show();
}

// Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                           const ob::GoalState* goal,
                           const ob::OptimizationObjective* obj,
                           const boost::property_map<ob::PlannerData::Graph::Type,
                           vertex_type_t>::type& plannerDataVertices)
{
  return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
}

void readPlannerDataStoredToGraphViz()
{
  std::cout << std::endl;
  std::cout << "Reading PlannerData" << std::endl;
  ros::NodeHandle node_handle_priv("~");
  cwru_davinci_grasp::DavinciNeedleGrasperBasePtr pNeedleGrasper(new cwru_davinci_grasp::DavinciNeedleGrasperBase(node_handle_priv,
                                                                                                                  "psm_one",
                                                                                                                  "psm_one_gripper"));

  const std::vector<cwru_davinci_grasp::GraspInfo> graspInfos = pNeedleGrasper->getAllPossibleNeedleGrasps();

  // Recreating the space information from the stored planner data instance
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, graspInfos.size() - 1, graspInfos));

  ompl::base::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.101);
  se3_xyz_bounds.setHigh(0, 0.101);
  se3_xyz_bounds.setLow(1, -0.1);
  se3_xyz_bounds.setHigh(1, 0.1);
  se3_xyz_bounds.setLow(2, -0.03);
  se3_xyz_bounds.setHigh(2, 0.18);

  hystsp->setSE3Bounds(se3_xyz_bounds);
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ob::PlannerDataStorage dataStorage;
  ob::PlannerData data(si);

  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  dataStorage.load((dataPath + "/../../../" + "HybridRRTPlannerData").c_str(), data);
  std::ofstream graphVizOutput(dataPath + "/../../../" + "HybridRRTPlannerData.dot");
  data.printGraphviz(graphVizOutput);
  graphVizOutput.close();

  // Re-extract the shortest path from the loaded planner data
  if (data.numStartVertices() > 0 && data.numGoalVertices() > 0)
  {
    // Create an optimization objective for optimizing path length in A*
    ob::PathLengthOptimizationObjective opt(si);

    // Computing the weights of all edges based on the state space distance
    // This is not done by default for efficiency
    data.computeEdgeWeights(opt);

    // Getting a handle to the raw Boost.Graph data
    ob::PlannerData::Graph::Type& graph = data.toBoostGraph();

    // Now we can apply any Boost.Graph algorithm.  How about A*!

    // create a predecessor map to store A* results in
    boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(data.numVertices());

    // Retrieve a property map with the PlannerDataVertex object pointers for quick lookup
    boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);

    // Run A* search over our planner data
    ob::GoalState goal(si);
    goal.setState(data.getGoalVertex(0).getState());
    ob::PlannerData::Graph::Vertex start = boost::vertex(data.getStartIndex(0), graph);
    boost::astar_search(graph, start,
                        [&goal, &opt, &vertices](ob::PlannerData::Graph::Vertex v1) { return distanceHeuristic(v1, &goal, &opt, vertices); },
                        boost::predecessor_map(prev).
                        distance_compare([&opt](ob::Cost c1, ob::Cost c2) { return opt.isCostBetterThan(c1, c2); }).
                        distance_combine([&opt](ob::Cost c1, ob::Cost c2) { return opt.combineCosts(c1, c2); }).
                        distance_inf(opt.infiniteCost()).
                        distance_zero(opt.identityCost()));

    // Extracting the path
    og::PathGeometric path(si);
    for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
         prev[pos] != pos;
         pos = prev[pos])
    {
      path.append(vertices[pos]->getState());
    }
    path.append(vertices[start]->getState());
    path.reverse();
    
    // print the path to screen
    std::ofstream outFile(dataPath + "/../../../" + "SolutionPathFromPlannerData.txt");
    path.print(std::cout);
    path.printAsMatrix(outFile);
    std::cout << "Found stored solution with " << path.getStateCount() << " states and length " << path.length() << std::endl;
  }
}

void readPlannerData()
{
  std::cout << std::endl;
  std::cout << "Reading PlannerData" << std::endl;

  // Recreating the space information from the stored planner data instance
  std::vector <cwru_davinci_grasp::GraspInfo> grasp_poses;
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size()-1, grasp_poses));

  ompl::base::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.101);
  se3_xyz_bounds.setHigh(0, 0.101);
  se3_xyz_bounds.setLow(1, -0.06);
  se3_xyz_bounds.setHigh(1, 0.09);
  se3_xyz_bounds.setLow(2, 0.266);
  se3_xyz_bounds.setHigh(2, 0.496);
  hystsp->setSE3Bounds(se3_xyz_bounds);

  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ob::PlannerDataStorage dataStorage;
  ob::PlannerData data(si);

  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  dataStorage.load((dataPath + "/../../../" + "HybridRRTPlannerData").c_str(), data);
  
//  ob::PlannerData::Graph::Vertex start = boost::vertex(data.getStartIndex(0), graph);
//  boost::depth_first_visit<>

  std::shared_ptr<ob::SE3StateSpace> subSE3SS(hystsp->as<ob::SE3StateSpace>(0));
  HyperPlaneDrawing hyperPlaneDrawer = HyperPlaneDrawing(subSE3SS->getBounds().low[0],
                                                         subSE3SS->getBounds().high[0],
                                                         subSE3SS->getBounds().low[1],
                                                         subSE3SS->getBounds().high[1],
                                                         subSE3SS->getBounds().low[2],
                                                         subSE3SS->getBounds().high[2],
                                                         2*147);

  hyperPlaneDrawer.drawSubCube();
  hyperPlaneDrawer.drawSubCube();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hybrid_planner_planning_data_visualization");
  ros::NodeHandle node_handle;
  ros::Duration(3.0).sleep();
   readPlannerDataStoredToGraphViz();
//  readPlannerData();
  return 0;
}
