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

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <boost/graph/depth_first_search.hpp>
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
  // readPlannerDataStoredToGraphViz();
  readPlannerData();
  return 0;
}
