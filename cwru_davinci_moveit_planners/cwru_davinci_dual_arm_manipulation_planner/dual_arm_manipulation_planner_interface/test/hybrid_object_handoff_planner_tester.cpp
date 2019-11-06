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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Su Lu*/
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <dual_arm_manipulation_planner_interface/hybrid_object_handoff_planner.h>
#include <boost/graph/astar_search.hpp>

#include <gtest/gtest.h>

using namespace dual_arm_manipulation_planner_interface;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace hybrid_planner_test
{
class HybridObjectHandoffPlannerTester : public HybridObjectHandoffPlanner
{
public:
  HybridObjectHandoffPlannerTester
  (
  const std::vector<cwru_davinci_grasp::GraspInfo>& graspInfo,
  const robot_model::RobotModelPtr& pRobotModel
  );

  virtual ~HybridObjectHandoffPlannerTester() {}

  void initializePlanner
  (
  const robot_model::RobotModelConstPtr& pRobotModel
  );

  void testConnectStates();

protected:
  void getSolutionPathFromData();

  ob::Cost distanceHeuristic
  (
  ob::PlannerData::Graph::Vertex v1,
  const ob::GoalState *goal,
  const ob::OptimizationObjective *obj,
  const boost::property_map<ob::PlannerData::Graph::Type,
  vertex_type_t>::type &plannerDataVertices
  );

protected:
  std::vector<ob::State *> m_SlnStates;

};

HybridObjectHandoffPlannerTester::HybridObjectHandoffPlannerTester
(
const std::vector<cwru_davinci_grasp::GraspInfo>& graspInfo,
const robot_model::RobotModelPtr& pRobotModel
)
 : HybridObjectHandoffPlanner(graspInfo)
{
  initializePlanner(pRobotModel);
}

void HybridObjectHandoffPlannerTester::initializePlanner
(
const robot_model::RobotModelConstPtr& pRobotModel
)
{
  ASSERT_TRUE(m_pHyStateSpace);
  setupSpaceInformation(m_pHyStateSpace, pRobotModel,"needle_r");
  EXPECT_TRUE(m_pSpaceInfor);
}

void HybridObjectHandoffPlannerTester::testConnectStates()
{
  getSolutionPathFromData();
  EXPECT_TRUE(!m_SlnStates.empty());


}

void HybridObjectHandoffPlannerTester::getSolutionPathFromData()
{
  ob::PlannerDataStorage dataStorage;
  ob::PlannerData data(m_pSpaceInfor);

  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  dataStorage.load((dataPath + "/../../../" + "HybridRRTPlannerData").c_str(), data);

  // Re-extract the shortest path from the loaded planner data
  if (data.numStartVertices() > 0 && data.numGoalVertices() > 0)
  {
    // Create an optimization objective for optimizing path length in A*
    ob::PathLengthOptimizationObjective opt(m_pSpaceInfor);

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
    ob::GoalState goal(m_pSpaceInfor);
    goal.setState(data.getGoalVertex(0).getState());
    ob::PlannerData::Graph::Vertex start = boost::vertex(data.getStartIndex(0), graph);
    boost::astar_search(graph, start,
                        [this, &goal, &opt, &vertices](ob::PlannerData::Graph::Vertex v1) { return distanceHeuristic(v1, &goal, &opt, vertices); },
                        boost::predecessor_map(prev).
                          distance_compare([&opt](ob::Cost c1, ob::Cost c2) { return opt.isCostBetterThan(c1, c2); }).
                          distance_combine([&opt](ob::Cost c1, ob::Cost c2) { return opt.combineCosts(c1, c2); }).
                          distance_inf(opt.infiniteCost()).
                          distance_zero(opt.identityCost()));

    // Extracting the path
    og::PathGeometric path(m_pSpaceInfor);
    for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
         prev[pos] != pos;
         pos = prev[pos])
    {
      path.append(vertices[pos]->getState());
    }
    path.append(vertices[start]->getState());
    path.reverse();

    m_SlnStates = path.getStates();
  }
}

// Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
ob::Cost HybridObjectHandoffPlannerTester::distanceHeuristic
(
ob::PlannerData::Graph::Vertex v1,
const ob::GoalState *goal,
const ob::OptimizationObjective *obj,
const boost::property_map<ob::PlannerData::Graph::Type,
vertex_type_t>::type &plannerDataVertices
)
{
  return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
}
}

