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

  void testPlanObjectTransit
  (
  const HybridObjectStateSpace::StateType* pHyFromState,
  const HybridObjectStateSpace::StateType* pHyToState
  );

  void testPlanHandoff
  (
  const HybridObjectStateSpace::StateType* pHyFromState,
  const HybridObjectStateSpace::StateType* pHyToState
  );

protected:
  void getSolutionPathFromData();

  void sameRobotState
  (
  const robot_state::RobotStateConstPtr& pRobotState,
  const std::vector<double>& jointValue,
  const std::string& supportGroup
  );

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
//  robot_model::RobotModelPtr m_pRobotModel;
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

  const HybridObjectStateSpace::StateType* pHyState0 = dynamic_cast<HybridObjectStateSpace::StateType*>(m_SlnStates[0]);
  const HybridObjectStateSpace::StateType* pHyState1 = dynamic_cast<HybridObjectStateSpace::StateType*>(m_SlnStates[1]);
  const HybridObjectStateSpace::StateType* pHyState2 = dynamic_cast<HybridObjectStateSpace::StateType*>(m_SlnStates[2]);
  const HybridObjectStateSpace::StateType* pHyState3 = dynamic_cast<HybridObjectStateSpace::StateType*>(m_SlnStates[3]);
  const HybridObjectStateSpace::StateType* pHyState4 = dynamic_cast<HybridObjectStateSpace::StateType*>(m_SlnStates[4]);
  const HybridObjectStateSpace::StateType* pHyState5 = dynamic_cast<HybridObjectStateSpace::StateType*>(m_SlnStates[5]);
  const HybridObjectStateSpace::StateType* pHyState6 = dynamic_cast<HybridObjectStateSpace::StateType*>(m_SlnStates[6]);
  const HybridObjectStateSpace::StateType* pHyState7 = dynamic_cast<HybridObjectStateSpace::StateType*>(m_SlnStates[7]);
  EXPECT_TRUE(pHyState0);
  EXPECT_TRUE(pHyState1);
  EXPECT_TRUE(pHyState2);
  EXPECT_TRUE(pHyState3);
  EXPECT_TRUE(pHyState4);
  EXPECT_TRUE(pHyState5);
  EXPECT_TRUE(pHyState6);
  EXPECT_TRUE(pHyState7);

  EXPECT_EQ(StateDiff::PoseDiffArmAndGraspSame, m_pHyStateSpace->checkStateDiff(pHyState0, pHyState1));
  EXPECT_EQ(StateDiff::ArmAndGraspDiffPoseSame, m_pHyStateSpace->checkStateDiff(pHyState1, pHyState2));
  EXPECT_EQ(StateDiff::AllSame,                 m_pHyStateSpace->checkStateDiff(pHyState2, pHyState3));
  EXPECT_EQ(StateDiff::PoseDiffArmAndGraspSame, m_pHyStateSpace->checkStateDiff(pHyState3, pHyState4));
  EXPECT_EQ(StateDiff::PoseDiffArmAndGraspSame, m_pHyStateSpace->checkStateDiff(pHyState4, pHyState5));
  EXPECT_EQ(StateDiff::PoseDiffArmAndGraspSame, m_pHyStateSpace->checkStateDiff(pHyState5, pHyState6));
  EXPECT_EQ(StateDiff::PoseDiffArmAndGraspSame, m_pHyStateSpace->checkStateDiff(pHyState6, pHyState7));

  testPlanObjectTransit(pHyState0, pHyState1);
  testPlanHandoff(pHyState1, pHyState2);
  testPlanObjectTransit(pHyState2, pHyState3);
  testPlanObjectTransit(pHyState3, pHyState4);
  testPlanObjectTransit(pHyState4, pHyState5);
  testPlanObjectTransit(pHyState5, pHyState6);
  testPlanObjectTransit(pHyState6, pHyState7);
}

void HybridObjectHandoffPlannerTester::testPlanObjectTransit
(
const HybridObjectStateSpace::StateType* pHyFromState,
const HybridObjectStateSpace::StateType* pHyToState
)
{
  MoveGroupJointTrajectory jntTrajectoryBtwStates;
  EXPECT_TRUE(planObjectTransit(pHyFromState, pHyToState, jntTrajectoryBtwStates));
  EXPECT_EQ(1, jntTrajectoryBtwStates.size());
  EXPECT_EQ(TrajectoryType::ObjectTransit, jntTrajectoryBtwStates[0].first);

  const std::string supportGroup = (pHyFromState->armIndex().value == 1) ? "psm_one" : "psm_two";
  EXPECT_EQ(supportGroup, jntTrajectoryBtwStates[0].second.begin()->first);

  const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  EXPECT_TRUE(pRobotFromState);
  EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyFromState, pRobotFromState));

  const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  EXPECT_TRUE(pRobotToState);
  EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyToState, pRobotToState));

  // construct two robot state from joint values
  sameRobotState(pRobotFromState, jntTrajectoryBtwStates[0].second.begin()->second[0], supportGroup);
  sameRobotState(pRobotToState, jntTrajectoryBtwStates[0].second.begin()->second.back(), supportGroup);
}

void HybridObjectHandoffPlannerTester::testPlanHandoff
(
const HybridObjectStateSpace::StateType* pHyFromState,
const HybridObjectStateSpace::StateType* pHyToState
)
{
  MoveGroupJointTrajectory jntTrajectoryBtwStates;
  EXPECT_TRUE(planHandoff(pHyFromState, pHyToState, jntTrajectoryBtwStates));
  EXPECT_EQ(4, jntTrajectoryBtwStates.size());
  EXPECT_EQ(TrajectoryType::SafePlaceToPreGrasp, jntTrajectoryBtwStates[0].first);
  EXPECT_EQ(TrajectoryType::PreGraspToGrasped,   jntTrajectoryBtwStates[1].first);
  EXPECT_EQ(TrajectoryType::GraspedToUngrasped,  jntTrajectoryBtwStates[2].first);
  EXPECT_EQ(TrajectoryType::UngrasedToSafePlace, jntTrajectoryBtwStates[3].first);

  const std::string fromSupportGroup = (pHyFromState->armIndex().value == 1) ? "psm_one" : "psm_two";
  const std::string toSupportGroup   = (pHyToState->armIndex().value == 1) ? "psm_one" : "psm_two";
  EXPECT_EQ(toSupportGroup,   jntTrajectoryBtwStates[0].second.begin()->first);
  EXPECT_EQ(toSupportGroup,   jntTrajectoryBtwStates[1].second.begin()->first);
  EXPECT_EQ(fromSupportGroup, jntTrajectoryBtwStates[2].second.begin()->first);
  EXPECT_EQ(fromSupportGroup, jntTrajectoryBtwStates[3].second.begin()->first);

  const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  EXPECT_TRUE(pRobotFromState);
  EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyFromState, pRobotFromState));

  const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  EXPECT_TRUE(pRobotToState);
  EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyToState, pRobotToState));

  std::vector<double> jointHomePosition = {0.0, 0.0, 0.016, 0.0, 0.0};
  EXPECT_EQ(jointHomePosition.size(), jntTrajectoryBtwStates[0].second.begin()->second[0].size());
  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jointHomePosition[i], jntTrajectoryBtwStates[0].second.begin()->second[0][i]);
  }

  sameRobotState(pRobotFromState, jntTrajectoryBtwStates[0].second.begin()->second[0], toSupportGroup);

  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jntTrajectoryBtwStates[0].second.begin()->second.back()[i], jntTrajectoryBtwStates[1].second.begin()->second[0][i]);
  }

  std::vector<double> graspedArmPosition;
  pRobotToState->copyJointGroupPositions(toSupportGroup, graspedArmPosition);
  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jntTrajectoryBtwStates[1].second.begin()->second.back()[i], graspedArmPosition[i]);
  }

  std::vector<double> graspedEefPosition;
  pRobotToState->copyJointGroupPositions(toSupportGroup + "_gripper", graspedEefPosition);
  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jntTrajectoryBtwStates[1].second[toSupportGroup + "_gripper"].back()[i], graspedArmPosition[i]);
  }

  std::vector<double> ungraspedArmPosition;
  pRobotFromState->copyJointGroupPositions(fromSupportGroup, ungraspedArmPosition);
  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jntTrajectoryBtwStates[2].second.begin()->second.begin()[i], ungraspedArmPosition[i]);
  }

  std::vector<double> ungraspedEefPosition;
  pRobotFromState->copyJointGroupPositions(fromSupportGroup + "_gripper", ungraspedEefPosition);
  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jntTrajectoryBtwStates[2].second[fromSupportGroup + "_gripper"].back()[i], 0.0);
  }

  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jntTrajectoryBtwStates[2].second.begin()->second.back()[i], jntTrajectoryBtwStates[3].second.begin()->second[0][i]);
  }

  EXPECT_EQ(jointHomePosition.size(), jntTrajectoryBtwStates[3].second.begin()->second.back().size());
  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jointHomePosition[i], jntTrajectoryBtwStates[3].second.begin()->second.back()[i]);
  }
  sameRobotState(pRobotToState, jntTrajectoryBtwStates[3].second.begin()->second.back(), fromSupportGroup);
}

void HybridObjectHandoffPlannerTester::sameRobotState
(
const robot_state::RobotStateConstPtr& pRobotState,
const std::vector<double>& jointValue,
const std::string& supportGroup
)
{
  const robot_state::RobotStatePtr pRobotStateCopy(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  pRobotStateCopy->setToDefaultValues();
  pRobotStateCopy->update();
  pRobotStateCopy->setJointGroupPositions(supportGroup, jointValue);
  m_pHyStateValidator->setMimicJointPositions(pRobotStateCopy, supportGroup);
  pRobotStateCopy->update();

  std::vector<double> jointValueVec;
  pRobotState->copyJointGroupPositions(supportGroup, jointValueVec);

  std::vector<double> jointValueVecCopy;
  pRobotStateCopy->copyJointGroupPositions(supportGroup, jointValueVecCopy);

  // compare joint values
  EXPECT_EQ(6, jointValueVec.size());
  EXPECT_EQ(jointValueVec.size(), jointValueVecCopy.size());
  for (std::size_t i = 0; i < jointValueVec.size(); ++i)
  {
    EXPECT_NEAR(jointValueVec[i], jointValueVecCopy[i], 1e-3);
  }

  // compare tool tip pose
  const Eigen::Affine3d tipPose = pRobotState->getGlobalLinkTransform(
  pRobotState->getJointModelGroup(supportGroup)->getOnlyOneEndEffectorTip());

  const Eigen::Affine3d tipPoseCopy = pRobotStateCopy->getGlobalLinkTransform(
  pRobotStateCopy->getJointModelGroup(supportGroup)->getOnlyOneEndEffectorTip());
  EXPECT_TRUE(tipPoseCopy.isApprox(tipPose, 1e-4));
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

