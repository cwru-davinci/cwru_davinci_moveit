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

  bool testSetupProblemDefinition
  (
  const ompl::base::State* start,
  const ompl::base::State* goal
  );

  bool testSetupPlanner
  (
  );

  ob::PlannerStatus::StatusType testSolve
  (
  const double solveTime
  );

  void testCartesianPath
  (
  const MoveGroupJointTrajectorySegment& jntTrajSeg
  );

  double distance
  (
  const Eigen::Affine3d& preTrans,
  const Eigen::Affine3d& postTrans
  );

  const ompl::base::SpaceInformationPtr& getSpaceInformation
  (
  ) const;

  bool testGetSolutionPathJointTrajectory
  (
  PathJointTrajectory& handoffPathJntTraj
  );

  // inline const std::shared_ptr<og::PathGeometric>& getSolutionPath
  // (
  // ) const
  // {
  //   return m_pPath;
  // }

  bool testValidateOriginalHandoffPath
  (
  const PathJointTrajectory& handoffPathJntTraj
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
  std::vector<ob::State* >                           m_SlnStates;
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
  m_pHyStateSpace->setArmIndexBounds(1, 2);
  ompl::base::RealVectorBounds se3Bounds(3);
  se3Bounds.setLow(0, -0.101);
  se3Bounds.setHigh(0, 0.101);
  se3Bounds.setLow(1, -0.1);
  se3Bounds.setHigh(1, 0.1);
  se3Bounds.setLow(2, -0.03);
  se3Bounds.setHigh(2, 0.18);

  m_pHyStateSpace->setSE3Bounds(se3Bounds);
  setupSpaceInformation(m_pHyStateSpace, pRobotModel,"needle_r");
  EXPECT_TRUE(m_pSpaceInfor);
  EXPECT_TRUE(m_pHyStateValidator);
}

bool HybridObjectHandoffPlannerTester::testSetupProblemDefinition
(
const ompl::base::State* start,
const ompl::base::State* goal
)
{
  setupProblemDefinition(start, goal);
  return m_pProblemDef.get();
}

bool HybridObjectHandoffPlannerTester::testSetupPlanner
(
)
{
  setupPlanner(100.0);
  return m_pRRTConnectPlanner.get();
}

ob::PlannerStatus::StatusType HybridObjectHandoffPlannerTester::testSolve
(
  const double solveTime
)
{
  return solve(solveTime);
}

bool HybridObjectHandoffPlannerTester::testGetSolutionPathJointTrajectory
(
PathJointTrajectory& handoffPathJntTraj
)
{
  getSolutionPathFromData();
  EXPECT_TRUE(!m_SlnStates.empty());

  std::vector<HybridObjectStateSpace::StateType*> m_SlnHYStates;
  m_SlnHYStates.resize(m_SlnStates.size());

  for (std::size_t i = 0; i < m_SlnStates.size(); ++i)
  {
    m_SlnHYStates[i] = m_SlnStates[i]->as<HybridObjectStateSpace::StateType>();
    m_SlnHYStates[i]->markValid();
    m_SlnHYStates[i]->setJointsComputed(true);
    EXPECT_TRUE(m_SlnHYStates[i]);
  }

  const size_t segments = m_SlnStates.size() - 1;

  handoffPathJntTraj.clear();
  handoffPathJntTraj.resize(segments);
  for (std::size_t i = 0; i < segments; ++i)
  {
    MoveGroupJointTrajectory jntTrajectoryBtwStates;
    if (!connectStates(m_SlnHYStates[i], m_SlnHYStates[i + 1], jntTrajectoryBtwStates))
    {
      return false;
    }
    handoffPathJntTraj[i] = jntTrajectoryBtwStates;
  }
  return true;
}

bool HybridObjectHandoffPlannerTester::testValidateOriginalHandoffPath
(
const PathJointTrajectory& handoffPathJntTraj
)
{
  EXPECT_TRUE(!m_SlnStates.empty());
  EXPECT_TRUE(m_pHyStateSpace);
  Eigen::Affine3d currentNeedlePose;
  std::vector<double> currentJointPosition;

  for (std::size_t i = 0; i < handoffPathJntTraj.size(); ++i)
  {
    if (handoffPathJntTraj[i].size() == 1)
      continue;

    const HybridObjectStateSpace::StateType* hyState = m_SlnStates[i]->as<HybridObjectStateSpace::StateType>();
    m_pHyStateSpace->copyJointValues(hyState, currentJointPosition);
    m_pHyStateSpace->se3ToEigen3d(hyState, currentNeedlePose);

    if (!validateOriginalHandoffPath(currentNeedlePose, currentJointPosition, handoffPathJntTraj[i]))
      return false;
  }
  return true;
}

void HybridObjectHandoffPlannerTester::testConnectStates()
{
  getSolutionPathFromData();
  EXPECT_TRUE(!m_SlnStates.empty());

  std::vector<HybridObjectStateSpace::StateType*> m_SlnHYStates;
  m_SlnHYStates.resize(m_SlnStates.size());

  for (std::size_t i = 0; i < m_SlnStates.size(); ++i)
  {
    m_SlnHYStates[i] = m_SlnStates[i]->as<HybridObjectStateSpace::StateType>();
    m_SlnHYStates[i]->markValid();
    m_SlnHYStates[i]->setJointsComputed(true);
    EXPECT_TRUE(m_SlnHYStates[i]);
  }

  for (std::size_t i = 0; i < m_SlnStates.size() - 1; ++i)
  {
    switch (m_pHyStateSpace->checkStateDiff(m_SlnHYStates[i], m_SlnHYStates[i+1]))
    {
      case StateDiff::AllSame:
        // progress to PoseDiffArmAndGraspSame
      case StateDiff::PoseDiffArmAndGraspSame:
        testPlanObjectTransit(m_SlnHYStates[i],  m_SlnHYStates[i+1]);
        break;
      case StateDiff::ArmAndGraspDiffPoseSame:
        testPlanHandoff(m_SlnHYStates[i],  m_SlnHYStates[i+1]);
        break;
      default:
        // should not be there
        break;
    }
  }
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

  testCartesianPath(jntTrajectoryBtwStates[0].second);

  //  Debug handoff state
  {
    const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
    EXPECT_TRUE(pRobotFromState);
    EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotState(pHyFromState, pRobotFromState));

    const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
    EXPECT_TRUE(pRobotToState);
    EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotState(pHyToState, pRobotToState));

    const robot_state::RobotStatePtr pHandoffState(new robot_state::RobotState(*pRobotFromState));

    m_pHyStateValidator->publishRobotState(*pRobotFromState);
    ros::Duration(3.0).sleep();
    m_pHyStateValidator->publishRobotState(*pRobotToState);
    ros::Duration(3.0).sleep();
  }
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

  EXPECT_EQ(1, jntTrajectoryBtwStates[0].second.size());
  EXPECT_EQ(2, jntTrajectoryBtwStates[1].second.size());
  EXPECT_EQ(2, jntTrajectoryBtwStates[2].second.size());
  EXPECT_EQ(1, jntTrajectoryBtwStates[3].second.size());

  std::map<MoveGroup, JointTrajectory>::const_iterator itr = jntTrajectoryBtwStates[1].second.begin();
  ++itr;
  EXPECT_EQ(toSupportGroup + "_gripper", itr->first);

  itr = jntTrajectoryBtwStates[2].second.begin();
  ++itr;
  EXPECT_EQ(fromSupportGroup + "_gripper", itr->first);

  const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  EXPECT_TRUE(pRobotFromState);
  EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyFromState, pRobotFromState));

  const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  EXPECT_TRUE(pRobotToState);
  EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyToState, pRobotToState));

  //  Debug handoff state
  {
    const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
    EXPECT_TRUE(pRobotFromState);
    EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotState(pHyFromState, pRobotFromState));

    const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
    EXPECT_TRUE(pRobotToState);
    EXPECT_TRUE(m_pHyStateValidator->hybridStateToRobotState(pHyToState, pRobotToState));

    const robot_state::RobotStatePtr pHandoffState(new robot_state::RobotState(*pRobotFromState));

    std::vector<double> gs_jt_position;
    pRobotToState->copyJointGroupPositions(toSupportGroup, gs_jt_position);
    pHandoffState->setJointGroupPositions(toSupportGroup, gs_jt_position);
    m_pHyStateValidator->setMimicJointPositions(pHandoffState, toSupportGroup);

    const moveit::core::AttachedBody* ss_needle_body = pRobotFromState->getAttachedBody("needle_r");
    const moveit::core::AttachedBody* gs_needle_body = pRobotToState->getAttachedBody("needle_r");

    std::set<std::string> touch_links = ss_needle_body->getTouchLinks();
    touch_links.insert(gs_needle_body->getTouchLinks().begin(), gs_needle_body->getTouchLinks().end());

    trajectory_msgs::JointTrajectory dettach_posture = ss_needle_body->getDetachPosture();
    trajectory_msgs::JointTrajectory gs_dettach_posture = gs_needle_body->getDetachPosture();

    dettach_posture.joint_names.insert(dettach_posture.joint_names.end(),
                                       gs_dettach_posture.joint_names.begin(),
                                       gs_dettach_posture.joint_names.end());

    dettach_posture.points.insert(dettach_posture.points.end(),
                                  gs_dettach_posture.points.begin(),
                                  gs_dettach_posture.points.end());


    pHandoffState->attachBody(gs_needle_body->getName(), gs_needle_body->getShapes(),
                              gs_needle_body->getFixedTransforms(), touch_links,
                              gs_needle_body->getAttachedLinkName(), gs_needle_body->getDetachPosture());
    pHandoffState->update();
    m_pHyStateValidator->publishRobotState(*pHandoffState);
    ros::Duration(3.0).sleep();
  }

  std::vector<double> jointHomePosition = {0.0, 0.0, 0.016, 0.0, 0.0, 0.0};
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
  for (std::size_t i = 0; i < graspedEefPosition.size(); ++i)
  {
    EXPECT_EQ(jntTrajectoryBtwStates[1].second[toSupportGroup + "_gripper"].back()[i], graspedEefPosition[i]);
  }

  std::vector<double> ungraspedArmPosition;
  pRobotFromState->copyJointGroupPositions(fromSupportGroup, ungraspedArmPosition);
  for (std::size_t i = 0; i < jointHomePosition.size(); ++i)
  {
    EXPECT_EQ(jntTrajectoryBtwStates[2].second.begin()->second.front()[i], ungraspedArmPosition[i]);
  }

  std::vector<double> ungraspedEefPosition;
  pRobotFromState->copyJointGroupPositions(fromSupportGroup + "_gripper", ungraspedEefPosition);
  for (std::size_t i = 0; i < ungraspedEefPosition.size(); ++i)
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

void HybridObjectHandoffPlannerTester::testCartesianPath
(
const MoveGroupJointTrajectorySegment& jntTrajSeg
)
{
  ASSERT_TRUE(!jntTrajSeg.empty());

  const robot_state::RobotStatePtr pRobotState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  pRobotState->setToDefaultValues();
  const std::string& moveGroup = jntTrajSeg.begin()->first;
  const JointTrajectory& jntTrajectory = jntTrajSeg.begin()->second;

  ASSERT_TRUE(!jntTrajectory.empty());

  pRobotState->setJointGroupPositions(moveGroup, jntTrajectory[0]);
  pRobotState->update();
  Eigen::Affine3d headTipPose = pRobotState->getGlobalLinkTransform(
  pRobotState->getJointModelGroup(moveGroup)->getOnlyOneEndEffectorTip());

  for (std::size_t i = 1 ; i < jntTrajectory.size() - 1; ++i)
  {
    pRobotState->setJointGroupPositions(moveGroup, jntTrajectory[i]);
    pRobotState->update();
    Eigen::Affine3d postTipPose = pRobotState->getGlobalLinkTransform(
    pRobotState->getJointModelGroup(moveGroup)->getOnlyOneEndEffectorTip());

    double distDiff = distance(headTipPose, postTipPose);
    EXPECT_LE(distDiff, 0.003);
    headTipPose = postTipPose;
  }
}

double HybridObjectHandoffPlannerTester::distance
(
const Eigen::Affine3d& preTrans,
const Eigen::Affine3d& postTrans
)
{
  Eigen::Vector3d translation = preTrans.translation() - postTrans.translation();
  double translationDist = translation.norm();

//  Eigen::Quaterniond preQua = Eigen::Quaterniond(preTrans.linear());
//  Eigen::Quaterniond postQua = Eigen::Quaterniond(postTrans.linear());
//  double rotationDist = fabs(postQua.angularDistance(preQua));

//  return translationDist + rotationDist;
  return translationDist;
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
    m_pSlnPath = std::make_shared<og::PathGeometric>(m_pSpaceInfor);
    for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
         prev[pos] != pos;
         pos = prev[pos])
    {
      m_pSlnPath->append(vertices[pos]->getState());
      printf("Vertex index along the solution path: %u\n", data.vertexIndex(*vertices[pos]));
    }
    m_pSlnPath->append(vertices[start]->getState());
    printf("Start vertex index along the solution path: %u\n", data.vertexIndex(*vertices[start]));
    m_pSlnPath->reverse();
    m_SlnStates = m_pSlnPath->getStates();
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

const ompl::base::SpaceInformationPtr& HybridObjectHandoffPlannerTester::getSpaceInformation
(
) const
{
  return m_pSpaceInfor;
}
}
