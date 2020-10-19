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
 * Description: This class is to do object handoff planning with given initial state and
 *              goal state, the result is robot joint trajectories for davinci dual-arm manipulators
 */

#include <dual_arm_manipulation_planner_interface/hybrid_object_handoff_planner.h>
#include <ompl/base/PlannerDataStorage.h>

using namespace dual_arm_manipulation_planner_interface;
namespace ob = ompl::base;
namespace og = ompl::geometric;

HybridObjectHandoffPlanner::HybridObjectHandoffPlanner
(
bool verbose
)
 : m_Verbose(verbose)
{
  m_pHyStateSpace = std::make_shared<HybridObjectStateSpace>();
}

HybridObjectHandoffPlanner::HybridObjectHandoffPlanner
(
const std::vector<cwru_davinci_grasp::GraspInfo>& graspInfo,
bool verbose
)
 : m_Verbose(verbose)
{
  setupStateSpace(graspInfo);
}

ob::PlannerStatus::StatusType HybridObjectHandoffPlanner::solve
(
const double solveTime
)
{
  if (m_pRRTConnectPlanner && m_pRRTConnectPlanner->isSetup())
  {
    auto startTs = std::chrono::high_resolution_clock::now();
    m_pProblemDef->print(std::cout);
    m_pSpaceInfor->printSettings(std::cout);

    m_Solved = m_pRRTConnectPlanner->ob::Planner::solve(solveTime);
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> timeUsed = finish - startTs;

    printf("HybridObjectHandoffPlanner: Handoff planner status is %s \n", m_Solved.asString().c_str());
    ob::PlannerData data(m_pSpaceInfor);
    m_pRRTConnectPlanner->getPlannerData(data);
    data.computeEdgeWeights();

    std::cout << "Found " << data.numVertices() << " vertices " << "\n";
    std::cout << "Found " << data.numEdges() << " edges " << "\n";
    std::cout << "Actual Planning Time is: " << timeUsed.count() << std::endl;

    return m_Solved;
  }

  m_Solved = ob::PlannerStatus::ABORT;
  printf("HybridObjectHandoffPlanner: Handoff planner status is %s \n", m_Solved.asString().c_str());
  return m_Solved;
}

bool HybridObjectHandoffPlanner::getSolutionPathJointTrajectory
(
PathJointTrajectory& handoffPathJntTraj
)
{
  if (!m_Solved || !m_pProblemDef->hasExactSolution())
  {
    m_pRRTConnectPlanner->clear();
    return false;
  }

  // m_pSlnPath = m_pProblemDef->getSolutionPath()->as<og::PathGeometric>();
  m_pSlnPath = std::make_shared<og::PathGeometric>(*(m_pProblemDef->getSolutionPath()->as<og::PathGeometric>()));

  if(!m_pSlnPath)
  {
    m_pRRTConnectPlanner->clear();
    return false;
  }

  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  std::ofstream outFile(dataPath + "/../../../" + "PathFound.txt");
  m_pSlnPath->printAsMatrix(outFile);
  outFile.close();

  ob::PlannerData data(m_pSpaceInfor);
  m_pRRTConnectPlanner->getPlannerData(data);
  ob::PlannerDataStorage dataStorage;
  dataStorage.store(data, (dataPath + "/../../../" + "HybridRRTPlannerData").c_str());

  const std::vector<ob::State*>& constSlnStates = m_pSlnPath->getStates();
  for (std::size_t i = 0; i < m_pSlnPath->getStateCount(); i++)
  {
    (constSlnStates[i]->as<HybridObjectStateSpace::StateType>())->markValid();
    (constSlnStates[i]->as<HybridObjectStateSpace::StateType>())->setJointsComputed(true);
  }

  const size_t segments = m_pSlnPath->getStateCount() - 1;

  handoffPathJntTraj.clear();
  handoffPathJntTraj.resize(segments);
  for (size_t i = 0; i < segments; ++i)
  {
    MoveGroupJointTrajectory jntTrajectoryBtwStates;
    if (!connectStates(constSlnStates[i], constSlnStates[i + 1], jntTrajectoryBtwStates))
    {
      m_pRRTConnectPlanner->clear();
      return false;
    }
    handoffPathJntTraj[i] = jntTrajectoryBtwStates;
  }

  m_pRRTConnectPlanner->clear();
  return true;
}

int HybridObjectHandoffPlanner::stepsBtwStates
(
const Eigen::Affine3d& needlePose,
const Eigen::Affine3d& targetPose
)
{
  Eigen::Quaterniond currentQua(needlePose.linear());
  Eigen::Quaterniond targetQua(targetPose.linear());

  double rotDistance = currentQua.angularDistance(targetQua);
  double transDistance = (targetPose.translation() - needlePose.translation()).norm();

  std::size_t transSteps = floor(transDistance / 0.001);
  std::size_t rotSteps = 0;

  std::size_t steps = std::max(transSteps, rotSteps) + 1;

  return steps;
}

double HybridObjectHandoffPlanner::distanceBtwPoses
(
const Eigen::Affine3d& currentPose,
const Eigen::Affine3d& targetPose
)
{
  Eigen::Quaterniond currentQua(currentPose.linear());
  Eigen::Quaterniond targetQua(targetPose.linear());

  double rotDistance = currentQua.angularDistance(targetQua);
  double transDistance = (targetPose.translation() - currentPose.translation()).norm();

  return rotDistance + transDistance;
}

void HybridObjectHandoffPlanner::setupStateSpace
(
const std::vector<cwru_davinci_grasp::GraspInfo>& graspInfo
)
{
  m_pHyStateSpace = std::make_shared<HybridObjectStateSpace>(graspInfo);
}

void HybridObjectHandoffPlanner::setupSpaceInformation
(
const HybridObjectStateSpacePtr& pHyStateSpace,
const robot_model::RobotModelConstPtr& pRobotModel,
const std::string& objectName
)
{
  m_pSpaceInfor = std::make_shared<ob::SpaceInformation>(m_pHyStateSpace);
  if (!m_pSpaceInfor)
  {
    m_pProblemDef = nullptr;
    m_pRRTConnectPlanner = nullptr;
    printf("HybridObjectHandoffPlanner: Failed to initialize space information \n");
    return;
  }
  m_pSpaceInfor->setStateValidityChecker(
  std::make_shared<HybridStateValidityChecker>(m_pSpaceInfor, pRobotModel, objectName));
  m_pSpaceInfor->setMotionValidator(
  std::make_shared<HybridMotionValidator>(m_pSpaceInfor, pRobotModel, objectName));

  m_pSpaceInfor->setup();

  m_pHyStateValidator = std::dynamic_pointer_cast<HybridStateValidityChecker>(m_pSpaceInfor->getStateValidityChecker());
}

void HybridObjectHandoffPlanner::setupProblemDefinition
(
const ompl::base::State* start,
const ompl::base::State* goal
)
{
  if (!m_pSpaceInfor->isValid(start) || !m_pSpaceInfor->isValid(goal))
  {
    m_pRRTConnectPlanner = nullptr;
    printf("HybridObjectHandoffPlanner: Either start or goal state is not valid \n");
    return;
  }

  m_pProblemDef = std::make_shared<ob::ProblemDefinition>(m_pSpaceInfor);
  if (!m_pProblemDef)
  {
    m_pRRTConnectPlanner = nullptr;
    printf("HybridObjectHandoffPlanner: Failed to initialize problem definition \n");
    return;
  }
  m_pProblemDef->setStartAndGoalStates(start, goal);
}

void HybridObjectHandoffPlanner::setupPlanner
(
const double maxDistance
)
{
  m_pRRTConnectPlanner = std::make_shared<og::RRTConnect>(m_pSpaceInfor);
  if (!m_pRRTConnectPlanner)
  {
    printf("HybridObjectHandoffPlanner: Failed to initialize RRTConnect planner \n");
    return;
  }
  m_pRRTConnectPlanner->setProblemDefinition(m_pProblemDef);
  m_pRRTConnectPlanner->setRange(maxDistance);
  if (!m_pRRTConnectPlanner->isSetup())
  {
    m_pRRTConnectPlanner->setup();
  }
}

bool HybridObjectHandoffPlanner::connectStates
(
const ompl::base::State* pFromState,
const ompl::base::State* pToState,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  const HybridObjectStateSpace::StateType* pHyFromState = dynamic_cast<const HybridObjectStateSpace::StateType*>(pFromState);
  const HybridObjectStateSpace::StateType* pHyToState = dynamic_cast<const HybridObjectStateSpace::StateType*>(pToState);

  if (!pHyFromState || !pHyToState)
  {
    printf("HybridObjectHandoffPlanner: Invalid states to be connected \n");
    return false;
  }

  bool hasTraj = false;
  switch (m_pHyStateSpace->checkStateDiff(pHyFromState, pHyToState))
  {
    case StateDiff::AllSame:
      // progress to PoseDiffArmAndGraspSame
    case StateDiff::PoseDiffArmAndGraspSame:
      hasTraj = planObjectTransit(pHyFromState, pHyToState, jntTrajectoryBtwStates);
      break;
    case StateDiff::ArmAndGraspDiffPoseSame:
      hasTraj = planHandoff(pHyFromState, pHyToState, jntTrajectoryBtwStates);
      break;
    default:
      // should not be there
      break;
  }
  return hasTraj;
}

bool HybridObjectHandoffPlanner::planObjectTransit
(
const HybridObjectStateSpace::StateType* pHyFromState,
const HybridObjectStateSpace::StateType* pHyToState,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  if (!m_pHyStateValidator)
  {
    printf("HybridObjectHandoffPlanner: Failed to connect states \n");
    return false;
  }

  const robot_state::RobotStatePtr pRobotFromState = std::make_shared<robot_state::RobotState>(m_pHyStateValidator->robotModel());
  if (!pRobotFromState || !m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyFromState, pRobotFromState))
  {
    printf("HybridObjectHandoffPlanner: Invalid FromState to be connected \n");
    return false;
  }
  const robot_state::RobotStatePtr pRobotToState = std::make_shared<robot_state::RobotState>(m_pHyStateValidator->robotModel());
  if (!pRobotToState || !m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyToState, pRobotToState))
  {
    printf("HybridObjectHandoffPlanner: Invalid ToState to be connected \n");
    return false;
  }

  const std::string supportGroup = (pHyFromState->armIndex().value == 1) ? "psm_one" : "psm_two";
  const robot_state::JointModelGroup* pSupportJntGroup = pRobotToState->getJointModelGroup(supportGroup);
  const moveit::core::LinkModel* pTipLink = pSupportJntGroup->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d toolTipPose = pRobotToState->getGlobalLinkTransform(pTipLink);

  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pRobotFromState->computeCartesianPath(pRobotFromState->getJointModelGroup(supportGroup),
                                                                    traj,
                                                                    pTipLink,
                                                                    toolTipPose,
                                                                    true,
                                                                    0.001,
                                                                    0.0);

  if (!(fabs(foundCartesianPath - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  JointTrajectory supportGroupJntTraj;
  supportGroupJntTraj.resize(traj.size());
  pRobotToState->copyJointGroupPositions(supportGroup, supportGroupJntTraj[traj.size() - 1]);

  for (std::size_t i = 0; i < traj.size() - 1; ++i)
  {
    traj[i]->update();
    traj[i]->copyJointGroupPositions(supportGroup, supportGroupJntTraj[i]);
  }

  MoveGroupJointTrajectorySegment jntTrajSegment = {{supportGroup, supportGroupJntTraj}};
  jntTrajectoryBtwStates.clear();
  jntTrajectoryBtwStates.resize(1);
  jntTrajectoryBtwStates[0] = std::make_pair(TrajectoryType::ObjectTransit, jntTrajSegment);
  return true;
}

bool HybridObjectHandoffPlanner::planHandoff
(
const HybridObjectStateSpace::StateType* pHyFromState,
const HybridObjectStateSpace::StateType* pHyToState,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  if (!m_pHyStateValidator)
  {
    printf("HybridObjectHandoffPlanner: Failed to connect states \n");
    return false;
  }

  const robot_state::RobotStatePtr pRobotFromState = std::make_shared<robot_state::RobotState>(m_pHyStateValidator->robotModel());
  if (!pRobotFromState || !m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyFromState, pRobotFromState))
  {
    printf("HybridObjectHandoffPlanner: Invalid FromState to be connected \n");
    return false;
  }
  const robot_state::RobotStatePtr pRobotToState = std::make_shared<robot_state::RobotState>(m_pHyStateValidator->robotModel());
  if (!pRobotToState || !m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyToState, pRobotToState))
  {
    printf("HybridObjectHandoffPlanner: Invalid ToState to be connected \n");
    return false;
  }

  // an intermediate state when needle is supporting by two grippers
  const robot_state::RobotStatePtr pHandoffRobotState = std::make_shared<robot_state::RobotState>(*pRobotFromState);
  if (!pHandoffRobotState)
  {
    return false;
  }

  const std::string fromSupportGroup = (pHyFromState->armIndex().value == 1) ? "psm_one" : "psm_two";
  const std::string toSupportGroup = (pHyToState->armIndex().value == 1) ? "psm_one" : "psm_two";
  std::vector<double> toSupportGroupJntPosition;
  pRobotToState->copyJointGroupPositions(toSupportGroup, toSupportGroupJntPosition);
  pHandoffRobotState->setJointGroupPositions(toSupportGroup, toSupportGroupJntPosition);
  m_pHyStateValidator->setMimicJointPositions(pHandoffRobotState, toSupportGroup);
  pHandoffRobotState->update();

  jntTrajectoryBtwStates.clear();
  jntTrajectoryBtwStates.resize(4);
  bool ableToGrasp = planNeedleGrasping(pRobotFromState, pHandoffRobotState, toSupportGroup, jntTrajectoryBtwStates);
  if (ableToGrasp)
  {
    bool ableToRelease = planNeedleReleasing(pHandoffRobotState, pRobotToState, fromSupportGroup,
                                             jntTrajectoryBtwStates);
    if (ableToRelease)
    {
      return true;
    }
  }
  return false;
}

bool HybridObjectHandoffPlanner::planNeedleGrasping
(
const robot_state::RobotStatePtr& pRobotFromState,
const robot_state::RobotStateConstPtr& pHandoffRobotState,
const std::string& toSupportGroup,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
// planning in a back order fashion
  robot_state::RobotStatePtr pPreGraspRobotState = std::make_shared<robot_state::RobotState>(*pRobotFromState);
  if (!planPreGraspStateToGraspedState(pPreGraspRobotState, pHandoffRobotState, toSupportGroup, jntTrajectoryBtwStates))
    return false;
  if (!planSafeStateToPreGraspState(pRobotFromState, pPreGraspRobotState, toSupportGroup, jntTrajectoryBtwStates))
    return false;
  return true;
}

bool HybridObjectHandoffPlanner::planPreGraspStateToGraspedState
(
robot_state::RobotStatePtr& pPreGraspRobotState,
const robot_state::RobotStateConstPtr& pHandoffRobotState,
const std::string& toSupportGroup,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  // make a pre_grasp_state
  const robot_state::JointModelGroup* pToSupportJntGroup = pPreGraspRobotState->getJointModelGroup(toSupportGroup);
  const moveit::core::LinkModel* pTipLink = pToSupportJntGroup->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d graspedToolTipPose = pHandoffRobotState->getGlobalLinkTransform(pTipLink);
  Eigen::Affine3d pregraspToolTipPose = graspedToolTipPose;
  Eigen::Vector3d unitApproachDir(0.0, 0.0, 1.0);  // grasp approach along the +z-axis of tip frame

  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridStateValidityChecker::isRobotStateValid,
                                                                                  this->m_pHyStateValidator,
                                                                                  boost::cref(*m_pHyStateValidator->getPlanningScene()),
                                                                                  boost::cref(toSupportGroup),
                                                                                  false, true, _1, _2, _3);
  bool foundIK = false;
  double distance = 0.01;
  while (distance <= 0.015)
  {
    Eigen::Vector3d approachDist = graspedToolTipPose.linear() * (distance * unitApproachDir);
    pregraspToolTipPose.translation() = graspedToolTipPose.translation() - approachDist;
    std::size_t attempts = 1;
    double timeout = 0.005;
    foundIK = pPreGraspRobotState->setFromIK(pToSupportJntGroup, pregraspToolTipPose, attempts, timeout, stateValidityCallbackFn);
    if (foundIK)
      break;
    distance += 0.001;
  }
  if (!foundIK)
  {
    return foundIK;
  }

  const std::string toSupportEefGroup = pToSupportJntGroup->getAttachedEndEffectorNames()[0];
  std::vector<double> toSupportEefGroupJntPosition;
  pPreGraspRobotState->copyJointGroupPositions(toSupportEefGroup, toSupportEefGroupJntPosition);

  for (std::size_t i = 0; i < toSupportEefGroupJntPosition.size(); ++i)
  {
    toSupportEefGroupJntPosition[i] = 0.5;
  }
  pPreGraspRobotState->setJointGroupPositions(toSupportEefGroup, toSupportEefGroupJntPosition);
  m_pHyStateValidator->setMimicJointPositions(pPreGraspRobotState, toSupportGroup);
  pPreGraspRobotState->update();

  std::vector<robot_state::RobotStatePtr> traj;
  double translationStepMax = 0.001, rotationStepMax = 0.0;
  moveit::core::MaxEEFStep maxStep(translationStepMax, rotationStepMax);
  moveit::core::JumpThreshold jumpThreshold;
  double foundCartesianPath = pPreGraspRobotState->computeCartesianPath(pPreGraspRobotState->getJointModelGroup(toSupportGroup),
                                                                        traj,
                                                                        pTipLink,
                                                                        graspedToolTipPose,
                                                                        true,
                                                                        maxStep,
                                                                        jumpThreshold);


  if (!(fabs(foundCartesianPath - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  JointTrajectory toSupportGroupJntTraj;
  toSupportGroupJntTraj.resize(traj.size());
  pHandoffRobotState->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[traj.size() - 1]);

  JointTrajectory toSupportEefGroupJntTraj;
  toSupportEefGroupJntTraj.resize(traj.size());
  pHandoffRobotState->copyJointGroupPositions(toSupportEefGroup, toSupportEefGroupJntTraj[traj.size() - 1]);
  toSupportEefGroupJntTraj[traj.size() - 1][0] = - 0.5;

  for (std::size_t i = 0; i < traj.size() - 1; ++i)
  {
    traj[i]->update();
    traj[i]->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[i]);
    traj[i]->copyJointGroupPositions(toSupportEefGroup, toSupportEefGroupJntTraj[i]);
  }

  MoveGroupJointTrajectorySegment preGraspToGraspedJntTrajSeg = {{toSupportGroup,    toSupportGroupJntTraj},
                                                                 {toSupportEefGroup, toSupportEefGroupJntTraj}};
  jntTrajectoryBtwStates[1] = std::make_pair(TrajectoryType::PreGraspToGrasped, preGraspToGraspedJntTrajSeg);

  pPreGraspRobotState = std::move(traj[0]);
  m_pHyStateValidator->setMimicJointPositions(pPreGraspRobotState, toSupportGroup);
  pPreGraspRobotState->update();

  return true;
}

bool HybridObjectHandoffPlanner::planSafeStateToPreGraspState
(
const robot_state::RobotStatePtr& pRobotFromState,
const robot_state::RobotStateConstPtr& pPreGraspRobotState,
const std::string& toSupportGroup,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  const robot_state::JointModelGroup* pToSupportJntGroup = pPreGraspRobotState->getJointModelGroup(toSupportGroup);
  const moveit::core::LinkModel* pTipLink = pToSupportJntGroup->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d toolTipPose = pPreGraspRobotState->getGlobalLinkTransform(pTipLink);
  const Eigen::Affine3d toolTipHomePose = pRobotFromState->getGlobalLinkTransform(pTipLink);

  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pRobotFromState->computeCartesianPath(pRobotFromState->getJointModelGroup(toSupportGroup),
                                                                    traj,
                                                                    pTipLink,
                                                                    toolTipPose,
                                                                    true,
                                                                    0.003,
                                                                    0.0);

  bool order = true; // forward: true, backward: false
  if (!(fabs(foundCartesianPath - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    *pRobotFromState = *pPreGraspRobotState;  // this will call RobotState copy() function make deep copy
    pRobotFromState->update();
    traj.clear();
    foundCartesianPath = pRobotFromState->computeCartesianPath(pRobotFromState->getJointModelGroup(toSupportGroup),
                                                               traj,
                                                               pTipLink,
                                                               toolTipHomePose,
                                                               true,
                                                               0.003,
                                                               0.0);

    if (!((foundCartesianPath - 0.8) >= std::numeric_limits<double>::epsilon()))
    {
      return false;
    }
    order = false;
  }

  JointTrajectory toSupportGroupJntTraj;
  toSupportGroupJntTraj.resize(traj.size());
  pPreGraspRobotState->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[traj.size() - 1]);

  if (!order)
  {
    for (std::size_t i = traj.size() - 1; i --> 0;)  // back order
    {
      traj[i]->update();
      traj[i]->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[i]);
    }
  }
  else
  {
    for (std::size_t i = 0; i < traj.size() - 1; ++i)
    {
      traj[i]->update();
      traj[i]->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[i]);
    }
  }

  MoveGroupJointTrajectorySegment safePlaceToPreGraspJntTrajSeg = {{toSupportGroup, toSupportGroupJntTraj}};
  jntTrajectoryBtwStates[0] = std::make_pair(TrajectoryType::SafePlaceToPreGrasp, safePlaceToPreGraspJntTrajSeg);

  return true;
}


bool HybridObjectHandoffPlanner::planNeedleReleasing
(
const robot_state::RobotStateConstPtr& pHandoffRobotState,
const robot_state::RobotStateConstPtr& pRobotToState,
const std::string& fromSupportGroup,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  robot_state::RobotStatePtr pUngraspedRobotState = std::make_shared<robot_state::RobotState>(*pRobotToState);

  if (!planGraspStateToUngraspedState(pHandoffRobotState, pUngraspedRobotState, fromSupportGroup, jntTrajectoryBtwStates))
    return false;
  if (!planUngraspedStateToSafeState(pUngraspedRobotState, pRobotToState, fromSupportGroup, jntTrajectoryBtwStates))
    return false;
  return true;
}

bool HybridObjectHandoffPlanner::planGraspStateToUngraspedState
(
const robot_state::RobotStateConstPtr& pHandoffRobotState,
robot_state::RobotStatePtr& pUngraspedRobotState,
const std::string& fromSupportGroup,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  // make a ungrasped_state
  const robot_state::JointModelGroup* pFromSupportJntGroup = pHandoffRobotState->getJointModelGroup(fromSupportGroup);
  const moveit::core::LinkModel* tipLink = pFromSupportJntGroup->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d graspedToolTipPose = pHandoffRobotState->getGlobalLinkTransform(tipLink);
  Eigen::Affine3d ungraspedToolTipPose = graspedToolTipPose;
  Eigen::Vector3d retreatDist(0.0, 0.0, -1.0);  // ungrasp retreat along the -z-axis of tip frame
  double distance = 0.01;
  retreatDist = graspedToolTipPose.linear() * (distance * retreatDist);
  ungraspedToolTipPose.translation() += retreatDist;

  std::vector<double> fromSupportGroupJntPosition;
  pHandoffRobotState->copyJointGroupPositions(fromSupportGroup, fromSupportGroupJntPosition);
  pUngraspedRobotState->setJointGroupPositions(fromSupportGroup, fromSupportGroupJntPosition);
  m_pHyStateValidator->setMimicJointPositions(pUngraspedRobotState, fromSupportGroup);

  const std::string fromSupportEefGroup = pFromSupportJntGroup->getAttachedEndEffectorNames()[0];
  std::vector<double> fromSupportEefGroupJntPosition;
  pUngraspedRobotState->copyJointGroupPositions(fromSupportEefGroup, fromSupportEefGroupJntPosition);

  for (std::size_t i = 0; i < fromSupportEefGroupJntPosition.size(); ++i)
  {
    fromSupportEefGroupJntPosition[i] = 0.5;
  }
  pUngraspedRobotState->setJointGroupPositions(fromSupportEefGroup, fromSupportEefGroupJntPosition);
  pUngraspedRobotState->update();

  if (!m_pHyStateValidator->noCollision(*pUngraspedRobotState, fromSupportGroup, false, true))
    return false;

  std::vector<robot_state::RobotStatePtr> traj;
  double translationStepMax = 0.001, rotationStepMax = 0.0;
  moveit::core::MaxEEFStep maxStep(translationStepMax, rotationStepMax);
  moveit::core::JumpThreshold jumpThreshold;

  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridStateValidityChecker::isRobotStateValid,
                                                                                  this->m_pHyStateValidator,
                                                                                  boost::cref(*m_pHyStateValidator->getPlanningScene()),
                                                                                  boost::cref(fromSupportGroup),
                                                                                  false, true, _1, _2, _3);
  double foundCartesianPath = pUngraspedRobotState->computeCartesianPath(pUngraspedRobotState->getJointModelGroup(fromSupportGroup),
                                                                         traj,
                                                                         tipLink,
                                                                         ungraspedToolTipPose,
                                                                         true,
                                                                         maxStep,
                                                                         jumpThreshold,
                                                                         stateValidityCallbackFn);

  JointTrajectory fromSupportGroupJntTraj;
  JointTrajectory fromSupportEefGroupJntTraj;
  std::vector<double> tmpJntTrajPoint;
  std::vector<double> tmpEefJntTrajPoint;

  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    traj[i]->copyJointGroupPositions(fromSupportGroup, tmpJntTrajPoint);
    traj[i]->copyJointGroupPositions(fromSupportEefGroup, tmpEefJntTrajPoint);
    fromSupportGroupJntTraj.push_back(tmpJntTrajPoint);
    fromSupportEefGroupJntTraj.push_back(tmpEefJntTrajPoint);
  }

  pUngraspedRobotState = std::move(traj.back());
  pUngraspedRobotState->setToDefaultValues(pUngraspedRobotState->getJointModelGroup(fromSupportEefGroup),
                                           fromSupportEefGroup + "_home");
  m_pHyStateValidator->setMimicJointPositions(pUngraspedRobotState, fromSupportGroup);
  pUngraspedRobotState->update();

  fromSupportEefGroupJntTraj.back()[0] = - 0.5;
  MoveGroupJointTrajectorySegment graspToUngraspedJntSeg = {{fromSupportGroup,    fromSupportGroupJntTraj},
                                                            {fromSupportEefGroup, fromSupportEefGroupJntTraj}};
  jntTrajectoryBtwStates[2] = std::make_pair(TrajectoryType::GraspedToUngrasped, graspToUngraspedJntSeg);

  return true;
}

bool HybridObjectHandoffPlanner::planUngraspedStateToSafeState
(
const robot_state::RobotStatePtr& pUngraspedRobotState,
const robot_state::RobotStateConstPtr& pRobotToState,
const std::string& fromSupportGroup,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  const robot_state::JointModelGroup* pFromSupportJntGroup = pRobotToState->getJointModelGroup(fromSupportGroup);
  const moveit::core::LinkModel* pTipLink = pFromSupportJntGroup->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d toolTipPose = pRobotToState->getGlobalLinkTransform(pTipLink);

  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pUngraspedRobotState->computeCartesianPath(pUngraspedRobotState->getJointModelGroup(fromSupportGroup),
                                                                         traj,
                                                                         pTipLink,
                                                                         toolTipPose,
                                                                         true,
                                                                         0.003,
                                                                         0.0);

  if (!((foundCartesianPath - 0.8) >= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  JointTrajectory fromSupportGroupJntTraj;
  fromSupportGroupJntTraj.resize(traj.size());
  pRobotToState->copyJointGroupPositions(fromSupportGroup, fromSupportGroupJntTraj[traj.size() - 1]);

  for (std::size_t i = 0; i < traj.size() - 1; ++i)
  {
    traj[i]->update();
    traj[i]->copyJointGroupPositions(fromSupportGroup, fromSupportGroupJntTraj[i]);
  }

  MoveGroupJointTrajectorySegment ungraspedToSafePlaceJntTrajSeg = {{fromSupportGroup, fromSupportGroupJntTraj}};
  jntTrajectoryBtwStates[3] = std::make_pair(TrajectoryType::UngrasedToSafePlace, ungraspedToSafePlaceJntTrajSeg);
  return true;
}

bool HybridObjectHandoffPlanner::localPlanObjectTransit
(
const std::vector<double>& currentJointPosition,
const Eigen::Affine3d& needlePose,
const int ithTraj,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  // Decide new collision free plan by partial planning
  ob::ScopedState<HybridObjectStateSpace> pHyFromState(m_pHyStateSpace);  // make a copy otherwise local modification will apply globally
  m_pHyStateSpace->copyState(pHyFromState.get(), m_pSlnPath->getState(ithTraj));
  m_pHyStateSpace->setJointValues(currentJointPosition, pHyFromState.get());
  m_pHyStateSpace->eigen3dToSE3(needlePose, pHyFromState.get());

  ob::ScopedState<HybridObjectStateSpace> pHyToState(m_pHyStateSpace);
  m_pHyStateSpace->copyState(pHyToState.get(), m_pSlnPath->getState(ithTraj + 1));
  m_pHyStateSpace->getSubspace(0)->copyState(pHyToState->components[0], pHyFromState->components[0]);
  pHyToState->clearKnownInformation();

  // first use local planner to check collision
  if (!m_pSpaceInfor->checkMotion(pHyFromState.get(), pHyToState.get()))
  {
    return false;
  }

  // if have new plan, then generate new handoff trajectories
  if (!planHandoff(pHyFromState.get(), pHyToState.get(), jntTrajectoryBtwStates))
  {
    return false;
  }

  return true;
}

bool HybridObjectHandoffPlanner::localPlanObjectTransfer
(
const Eigen::Affine3d& currentNeedlePose,
const Eigen::Affine3d& targetNeedlePose,
const std::vector<double>& currentJointPosition,
const std::string& supportGroup,
MoveGroupJointTrajectorySegment& jntTrajSeg,
double& time
)
{
  const robot_state::RobotStatePtr pCurrentRobotState = std::make_shared<robot_state::RobotState>(m_pHyStateValidator->robotModel());
  if (!pCurrentRobotState)
  {
    return false;
  }

  const moveit::core::LinkModel* pTipLink = pCurrentRobotState->getJointModelGroup(supportGroup)->getOnlyOneEndEffectorTip();
  pCurrentRobotState->setToDefaultValues();
  pCurrentRobotState->setJointGroupPositions(supportGroup, currentJointPosition);
  m_pHyStateValidator->setMimicJointPositions(pCurrentRobotState, supportGroup);
  pCurrentRobotState->update();
  const Eigen::Affine3d currentToolTipPose = pCurrentRobotState->getGlobalLinkTransform(pTipLink);
  const Eigen::Affine3d currentGrasp = currentToolTipPose.inverse() * currentNeedlePose;

  Eigen::Quaterniond currentQua(currentNeedlePose.linear());
  Eigen::Quaterniond targetQua(targetNeedlePose.linear());

  double percentage = (distanceBtwPoses(currentNeedlePose, targetNeedlePose) <= 0.5) ? 1.0 : 0.5;

  // Eigen::Affine3d toolTipPose(currentQua.slerp(percentage, targetQua));
  Eigen::Affine3d interNeedlePose(currentQua.slerp(percentage, targetQua));
  interNeedlePose.translation() = percentage * targetNeedlePose.translation() + (1 - percentage) * currentNeedlePose.translation();

  time = (((currentNeedlePose.translation() - interNeedlePose.translation()).norm()) / 0.001) * 0.2;
  moveit::core::AttachedBody *pNeedleModel = m_pHyStateValidator->createAttachedBody(supportGroup, "needle_r", currentGrasp);
  pCurrentRobotState->attachBody(pNeedleModel);
  pCurrentRobotState->update();
  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pCurrentRobotState->computeCartesianPath(pCurrentRobotState->getJointModelGroup(supportGroup),
                                                                       traj,
                                                                       pTipLink,
                                                                       interNeedlePose * currentGrasp.inverse(),
                                                                       true,
                                                                       0.001,
                                                                       0.0);
  // double foundCartesianPath = pCurrentRobotState->computeCartesianPath(pCurrentRobotState->getJointModelGroup(supportGroup),
  //                                                                      traj,
  //                                                                      pTipLink,
  //                                                                      interNeedlePose * currentGrasp.inverse(),
  //                                                                      true,
  //                                                                      moveit::core::MaxEEFStep(0.0005, 0.01),
  //                                                                      moveit::core::JumpThreshold(0.0));

  if (!(fabs(foundCartesianPath - 0.0) >= 1e-2))
  {
    ROS_WARN("LocalPlanObjectTransfer: Failed no IK solution");
    return false;
  }

  JointTrajectory supportGroupJntTraj;
  supportGroupJntTraj.resize(traj.size());

  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    m_pHyStateValidator->setMimicJointPositions(traj[i], supportGroup);
    traj[i]->update();
    if (!m_pHyStateValidator->noCollision(*traj[i], "", true, true))
    {
      ROS_WARN("LocalPlanObjectTransfer: Failed collision detected");
      return false;
    }

    traj[i]->copyJointGroupPositions(supportGroup, supportGroupJntTraj[i]);
  }

  jntTrajSeg = {{supportGroup, supportGroupJntTraj}};

  return true;
}

bool HybridObjectHandoffPlanner::validateOriginalHandoffPath
(
const Eigen::Affine3d& currentNeedlePose,
const std::vector<double>& currentJointPosition,
const MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  const std::string toSupportGroup = jntTrajectoryBtwStates[0].second.begin()->first;
  const std::string toRestGroup    = jntTrajectoryBtwStates[2].second.begin()->first;

  if (!m_pHyStateValidator)
  {
    ROS_ERROR("HybridObjectHandoffPlanner: Failed to validate original HndF path, invalid validity checker");
  }

  // doing some preparation
  robot_state::RobotStatePtr pCurrentRobotState = std::make_shared<robot_state::RobotState>(m_pHyStateValidator->robotModel());
  pCurrentRobotState->setToDefaultValues();
  pCurrentRobotState->setJointGroupPositions(toRestGroup, currentJointPosition);
  m_pHyStateValidator->setMimicJointPositions(pCurrentRobotState, toRestGroup);
  pCurrentRobotState->update();
  const Eigen::Affine3d toRestGroupInitialToolTipPose = pCurrentRobotState->getGlobalLinkTransform(pCurrentRobotState->getJointModelGroup(toRestGroup)->getOnlyOneEndEffectorTip());
  const Eigen::Affine3d toRestGroupCurrentGrasp       = toRestGroupInitialToolTipPose.inverse() * currentNeedlePose;
  // attach object to supporting joint group of robot
  pCurrentRobotState->attachBody(m_pHyStateValidator->createAttachedBody(toRestGroup,
                                                                         m_pHyStateValidator->objectName(),
                                                                         toRestGroupCurrentGrasp));
  pCurrentRobotState->update();

  if (!m_pHyStateValidator->validateTrajectory(toSupportGroup, pCurrentRobotState, jntTrajectoryBtwStates[0].second.begin()->second, 0.0, true))
    return false;

  if (!m_pHyStateValidator->validateTrajectory(toSupportGroup, pCurrentRobotState, jntTrajectoryBtwStates[1].second.begin()->second, 0.5, true))
    return false;

  m_pHyStateValidator->setJawPosition(0.0, toSupportGroup, pCurrentRobotState);

  // detect if needle is grasped by toSupportGroup
  if (!m_pHyStateValidator->detechNeedleGrasped(*pCurrentRobotState, toSupportGroup, true))
  {
    ROS_ERROR("HybridObjectHandoffPlanner: Validation of original handoff path fails because it can NOT reach to grasp");
    return false;
  }

  pCurrentRobotState->clearAttachedBodies();
  pCurrentRobotState->update();

  const Eigen::Affine3d toSupportGroupCurrentToolTipPose = pCurrentRobotState->getGlobalLinkTransform(pCurrentRobotState->getJointModelGroup(toSupportGroup)->getOnlyOneEndEffectorTip());
  const Eigen::Affine3d toSupportGroupCurrentGrasp       = toSupportGroupCurrentToolTipPose.inverse() * currentNeedlePose;
  // attach object to supporting joint group of robot
  pCurrentRobotState->attachBody(m_pHyStateValidator->createAttachedBody(toSupportGroup,
                                                                         m_pHyStateValidator->objectName(),
                                                                         toSupportGroupCurrentGrasp));

  m_pHyStateValidator->setJawPosition(0.5, toRestGroup, pCurrentRobotState);

  MoveGroupJointTrajectorySegment newPathToUngrasp;
  if (replanGraspToUngrasped(newPathToUngrasp, toRestGroupInitialToolTipPose, pCurrentRobotState, toRestGroup))
  {
    const_cast<MoveGroupJointTrajectorySegment&>(jntTrajectoryBtwStates[2].second) = newPathToUngrasp;

    MoveGroupJointTrajectorySegment newPathToHome;
    if (replanUnGraspToSafePlace(newPathToHome, jntTrajectoryBtwStates[3].second.begin()->second.back(),
                                 pCurrentRobotState, toRestGroup))
    {
      const_cast<MoveGroupJointTrajectorySegment&>(jntTrajectoryBtwStates[3].second) = newPathToHome;
      return true;
    }

    if (!m_pHyStateValidator->validateTrajectory(toSupportGroup, pCurrentRobotState, jntTrajectoryBtwStates[3].second.begin()->second, 0.0, true))
      return false;

    return true;
  }

  if (!m_pHyStateValidator->validateTrajectory(toSupportGroup, pCurrentRobotState, jntTrajectoryBtwStates[2].second.begin()->second, 0.5, true, false))
    return false;

  if (!m_pHyStateValidator->validateTrajectory(toSupportGroup, pCurrentRobotState, jntTrajectoryBtwStates[3].second.begin()->second, 0.0, true))
    return false;

  return true;
}

bool HybridObjectHandoffPlanner::replanGraspToUngrasped
(
MoveGroupJointTrajectorySegment& newPathToUngrasp,
const Eigen::Affine3d& toRestGroupInitialToolTipPose,
robot_state::RobotStatePtr& pCurrentRobotState,
const std::string& toRestGroup
)
{
  Eigen::Affine3d ungraspedToolTipPose = toRestGroupInitialToolTipPose;
  Eigen::Vector3d retreatDir(0.0, 0.0, -1.0);  // ungrasp retreat along the -z-axis of tip frame
  double distance = 0.01;
  retreatDir = ungraspedToolTipPose.linear() * (distance * retreatDir);
  ungraspedToolTipPose.translation() += retreatDir;

  if (!m_pHyStateValidator->noCollision(*pCurrentRobotState, toRestGroup, false, true))
    return false;

  std::vector<robot_state::RobotStatePtr> traj;
  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridStateValidityChecker::isRobotStateValid,
                                                                                  this->m_pHyStateValidator,
                                                                                  boost::cref(*m_pHyStateValidator->getPlanningScene()),
                                                                                  boost::cref(toRestGroup),
                                                                                  false, true, _1, _2, _3);
  double foundCartesianPath = pCurrentRobotState->computeCartesianPath(pCurrentRobotState->getJointModelGroup(toRestGroup),
                                                                       traj,
                                                                       pCurrentRobotState->getJointModelGroup(toRestGroup)->getOnlyOneEndEffectorTip(),
                                                                       ungraspedToolTipPose,
                                                                       true,
                                                                       0.001,
                                                                       0.0,
                                                                       stateValidityCallbackFn);
  JointTrajectory fromSupportGroupJntTraj;
  JointTrajectory fromSupportEefGroupJntTraj;
  std::vector<double> tmpJntTrajPoint;
  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    traj[i]->copyJointGroupPositions(toRestGroup, tmpJntTrajPoint);
    fromSupportGroupJntTraj.push_back(tmpJntTrajPoint);
  }

  pCurrentRobotState = std::move(traj.back());
  m_pHyStateValidator->setJawPosition(0.0, toRestGroup, pCurrentRobotState);
  newPathToUngrasp = {{toRestGroup, fromSupportGroupJntTraj}};

  return true;
}

bool HybridObjectHandoffPlanner::replanUnGraspToSafePlace
(
MoveGroupJointTrajectorySegment& newPathToHome,
const std::vector<double>& toRestJointPosition,
const robot_state::RobotStatePtr& pCurrentRobotState,
const std::string& toRestGroup
)
{
  robot_state::GroupStateValidityCallbackFn stateValidityCallbackFn = boost::bind(&HybridStateValidityChecker::isRobotStateValid,
                                                                                  this->m_pHyStateValidator,
                                                                                  boost::cref(*m_pHyStateValidator->getPlanningScene()),
                                                                                  boost::cref(toRestGroup),
                                                                                  true, true, _1, _2, _3);

  const robot_state::RobotStatePtr pToRestRobotState = std::make_shared<robot_state::RobotState>(m_pHyStateValidator->robotModel());
  pToRestRobotState->setToDefaultValues();
  pToRestRobotState->setJointGroupPositions(toRestGroup, toRestJointPosition);
  m_pHyStateValidator->setMimicJointPositions(pToRestRobotState, toRestGroup);
  pToRestRobotState->update();
  const Eigen::Affine3d toRestHomePose = pToRestRobotState->getGlobalLinkTransform(pToRestRobotState->getJointModelGroup(toRestGroup)->getOnlyOneEndEffectorTip());

  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pCurrentRobotState->computeCartesianPath(pCurrentRobotState->getJointModelGroup(toRestGroup),
                                                                       traj,
                                                                       pCurrentRobotState->getJointModelGroup(toRestGroup)->getOnlyOneEndEffectorTip(),
                                                                       toRestHomePose,
                                                                       true,
                                                                       0.003,
                                                                       0.0,
                                                                       stateValidityCallbackFn);

  if (!((foundCartesianPath - 0.8) >= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  JointTrajectory fromSupportGroupJntTraj;
  fromSupportGroupJntTraj.resize(traj.size());

  for (std::size_t i = 0; i < traj.size(); ++i)
  {
    traj[i]->copyJointGroupPositions(toRestGroup, fromSupportGroupJntTraj[i]);
  }

  newPathToHome = {{toRestGroup, fromSupportGroupJntTraj}};
  return true;
}
// void HybridObjectHandoffPlanner::getCurrentGrasp
// (
// Eigen::Affine3d& currentGrasp,
// const Eigen::Affine3d& currentNeedlePose,
// const std::string& supportGroup,
// const robot_state::RobotStatePtr& pCurrentRobotState
// )
// {
//   const moveit::core::LinkModel* pTipLink = pCurrentRobotState->getJointModelGroup(supportGroup)->getOnlyOneEndEffectorTip();
//   const Eigen::Affine3d currentToolTipPose = pCurrentRobotState->getGlobalLinkTransform(pTipLink);
//   currentGrasp = currentToolTipPose.inverse() * currentNeedlePose;
// }
