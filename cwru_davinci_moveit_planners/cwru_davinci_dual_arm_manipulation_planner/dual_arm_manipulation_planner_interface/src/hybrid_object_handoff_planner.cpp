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
    m_Solved = m_pRRTConnectPlanner->ob::Planner::solve(solveTime);
    printf("HybridObjectHandoffPlanner: Handoff planner status is %s", m_Solved.asString().c_str());
    return m_Solved;
  }

  m_Solved = ob::PlannerStatus::ABORT;
  printf("HybridObjectHandoffPlanner: Handoff planner status is %s", m_Solved.asString().c_str());
  return m_Solved;
}

bool HybridObjectHandoffPlanner::getSolutionPathJointTrajectory
(
PathJointTrajectory& handoffPathJntTraj
)
{
  if (!m_Solved || !m_pProblemDef->hasExactSolution())
  {
    return false;
  }

  og::PathGeometric* slnPath = m_pProblemDef->getSolutionPath()->as<og::PathGeometric>();

  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  std::ofstream outFile(dataPath + "/../../../" + "PathFound.txt");
  slnPath->printAsMatrix(outFile);
  outFile.close();

  ob::PlannerData data(m_pSpaceInfor);
  m_pRRTConnectPlanner->getPlannerData(data);
  ob::PlannerDataStorage dataStorage;
  dataStorage.store(data, (dataPath + "/../../../" + "HybridRRTPlannerData").c_str());

  const std::vector<ob::State*>& constSlnStates = slnPath->getStates();
  for (std::size_t i = 0; i < slnPath->getStateCount(); i++)
  {
    (constSlnStates[i]->as<HybridObjectStateSpace::StateType>())->markValid();
    (constSlnStates[i]->as<HybridObjectStateSpace::StateType>())->setJointsComputed(true);
  }

  const size_t segments = slnPath->getStateCount() - 1;

  handoffPathJntTraj.clear();
  handoffPathJntTraj.resize(segments);
  for (size_t i = 0; i < segments; ++i)
  {
    MoveGroupJointTrajectory jntTrajectoryBtwStates;
    if (!connectStates(constSlnStates[i], constSlnStates[i + 1], jntTrajectoryBtwStates))
    {
      return false;
    }
    handoffPathJntTraj[i] = jntTrajectoryBtwStates;
  }
  return true;
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
    printf("HybridObjectHandoffPlanner: Failed to initialize space information");
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
    printf("HybridObjectHandoffPlanner: Either start or goal state is not valid");
    return;
  }

  m_pProblemDef = std::make_shared<ob::ProblemDefinition>(m_pSpaceInfor);
  if (!m_pProblemDef)
  {
    m_pRRTConnectPlanner = nullptr;
    printf("HybridObjectHandoffPlanner: Failed to initialize problem definition");
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
    printf("HybridObjectHandoffPlanner: Failed to initialize RRTConnect planner");
    return;
  }
  m_pRRTConnectPlanner->setProblemDefinition(m_pProblemDef);
  m_pRRTConnectPlanner->setRange(maxDistance);
  m_pRRTConnectPlanner->setup();
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
    printf("HybridObjectHandoffPlanner: Invalid states to be connected");
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
    printf("HybridObjectHandoffPlanner: Failed to connect states");
    return false;
  }

  const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  if (!pRobotFromState || !m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyFromState, pRobotFromState))
  {
    printf("HybridObjectHandoffPlanner: Invalid FromState to be connected");
    return false;
  }
  const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  if (!pRobotToState || !m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyToState, pRobotToState))
  {
    printf("HybridObjectHandoffPlanner: Invalid ToState to be connected");
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

  if (!((foundCartesianPath - 1.0) <= std::numeric_limits<double>::epsilon()))
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
    printf("HybridObjectHandoffPlanner: Failed to connect states");
    return false;
  }

  const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  if (!pRobotFromState || !m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyFromState, pRobotFromState))
  {
    printf("HybridObjectHandoffPlanner: Invalid FromState to be connected");
    return false;
  }
  const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  if (!pRobotToState || !m_pHyStateValidator->hybridStateToRobotStateNoAttachedObject(pHyToState, pRobotToState))
  {
    printf("HybridObjectHandoffPlanner: Invalid ToState to be connected");
    return false;
  }

  // an intermediate state when needle is supporting by two grippers
  const robot_state::RobotStatePtr pHandoffRobotState(new robot_state::RobotState(*pRobotFromState));
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
  robot_state::RobotStatePtr pPreGraspRobotState(new robot_state::RobotState(*pRobotFromState));
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
  m_pHyStateValidator->publishRobotState(*pPreGraspRobotState);
// make a pre_grasp_state
  const robot_state::JointModelGroup* pToSupportJntGroup = pPreGraspRobotState->getJointModelGroup(toSupportGroup);
  const moveit::core::LinkModel* pTipLink = pToSupportJntGroup->getOnlyOneEndEffectorTip();

  const Eigen::Affine3d graspedToolTipPose = pHandoffRobotState->getGlobalLinkTransform(pTipLink);
  Eigen::Affine3d pregraspToolTipPose = graspedToolTipPose;
  Eigen::Vector3d unitApproachDir(0.0, 0.0, 1.0);  // grasp approach along the +z-axis of tip frame

  bool foundIK = false;
  double distance = 0.008;
  while (distance <= 0.013)
  {
    Eigen::Vector3d approachDist = graspedToolTipPose.linear() * (distance * unitApproachDir);
    pregraspToolTipPose.translation() = graspedToolTipPose.translation() - approachDist;
    std::size_t attempts = 1;
    double timeout = 0.1;
    foundIK = pPreGraspRobotState->setFromIK(pToSupportJntGroup, pregraspToolTipPose, attempts, timeout);
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

  m_pHyStateValidator->publishRobotState(*pPreGraspRobotState);

  std::vector<robot_state::RobotStatePtr> traj;
  double translationStepMax = 0.001, rotationStepMax = 0.0;
  moveit::core::MaxEEFStep maxStep(translationStepMax, rotationStepMax);
  moveit::core::JumpThreshold jumpThreshold;
  double foundCartesianPath = pPreGraspRobotState->computeCartesianPath(pToSupportJntGroup,
                                                                        traj,
                                                                        pTipLink,
                                                                        graspedToolTipPose,
                                                                        true,
                                                                        maxStep,
                                                                        jumpThreshold);


  if (!((foundCartesianPath - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  // removable
  pPreGraspRobotState->update();
  m_pHyStateValidator->publishRobotState(*pPreGraspRobotState);

  JointTrajectory toSupportGroupJntTraj;
  toSupportGroupJntTraj.resize(traj.size());
  pHandoffRobotState->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[traj.size() - 1]);

  JointTrajectory toSupportEefGroupJntTraj;
  toSupportEefGroupJntTraj.resize(traj.size());
  pHandoffRobotState->copyJointGroupPositions(toSupportEefGroup, toSupportEefGroupJntTraj[traj.size() - 1]);

  for (std::size_t i = 0; i < traj.size() - 1; ++i)
  {
    traj[i]->update();
    traj[i]->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[i]);
    traj[i]->copyJointGroupPositions(toSupportEefGroup, toSupportEefGroupJntTraj[i]);
  }

  MoveGroupJointTrajectorySegment preGraspToGraspedJntTrajSeg = {{toSupportGroup,    toSupportGroupJntTraj},
                                                                 {toSupportEefGroup, toSupportEefGroupJntTraj}};
  jntTrajectoryBtwStates[1] = std::make_pair(TrajectoryType::PreGraspToGrasped, preGraspToGraspedJntTrajSeg);

  pPreGraspRobotState.reset(new robot_state::RobotState(*traj[0]));
  m_pHyStateValidator->setMimicJointPositions(pPreGraspRobotState, toSupportGroup);
  pPreGraspRobotState->update();

  m_pHyStateValidator->publishRobotState(*pPreGraspRobotState);

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
  m_pHyStateValidator->publishRobotState(*pPreGraspRobotState);

  const robot_state::JointModelGroup* pToSupportJntGroup = pPreGraspRobotState->getJointModelGroup(toSupportGroup);
  const moveit::core::LinkModel* pTipLink = pToSupportJntGroup->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d toolTipPose = pPreGraspRobotState->getGlobalLinkTransform(pTipLink);

  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pRobotFromState->computeCartesianPath(pRobotFromState->getJointModelGroup(toSupportGroup),
                                                                    traj,
                                                                    pTipLink,
                                                                    toolTipPose,
                                                                    true,
                                                                    0.003,
                                                                    0.0);

  if (!((foundCartesianPath - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  pRobotFromState->update();
  m_pHyStateValidator->publishRobotState(*pRobotFromState);

  JointTrajectory toSupportGroupJntTraj;
  toSupportGroupJntTraj.resize(traj.size());
  pPreGraspRobotState->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[traj.size() - 1]);

  for (std::size_t i = 0; i < traj.size() - 1; ++i)
  {
    traj[i]->update();
    traj[i]->copyJointGroupPositions(toSupportGroup, toSupportGroupJntTraj[i]);
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
  robot_state::RobotStatePtr pUngraspedRobotState(new robot_state::RobotState(*pRobotToState));

  planGraspStateToUngraspedState(pHandoffRobotState, pUngraspedRobotState, fromSupportGroup, jntTrajectoryBtwStates);
  m_pHyStateValidator->publishRobotState(*pUngraspedRobotState);
  if (!planUngraspedStateToSafeState(pUngraspedRobotState, pRobotToState, fromSupportGroup, jntTrajectoryBtwStates))
    return false;
  return true;
}

bool HybridObjectHandoffPlanner::planGraspStateToUngraspedState
(
const robot_state::RobotStateConstPtr& pHandoffRobotState,
const robot_state::RobotStatePtr& pUngraspedRobotState,
const std::string& fromSupportGroup,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
// make a ungrasped_state
  m_pHyStateValidator->publishRobotState(*pHandoffRobotState);
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

  m_pHyStateValidator->publishRobotState(*pUngraspedRobotState);

  std::vector<robot_state::RobotStatePtr> traj;
  double translationStepMax = 0.001, rotationStepMax = 0.0;
  moveit::core::MaxEEFStep maxStep(translationStepMax, rotationStepMax);
  moveit::core::JumpThreshold jumpThreshold;
  double foundCartesianPath = pUngraspedRobotState->computeCartesianPath(pFromSupportJntGroup,
                                                                         traj,
                                                                         tipLink,
                                                                         ungraspedToolTipPose,
                                                                         true,
                                                                         maxStep,
                                                                         jumpThreshold);

  if (!((foundCartesianPath - 0.9) >= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  m_pHyStateValidator->setMimicJointPositions(pUngraspedRobotState, fromSupportGroup);
  pUngraspedRobotState->setToDefaultValues(pUngraspedRobotState->getJointModelGroup(fromSupportEefGroup),
                                           fromSupportEefGroup + "_home");
  pUngraspedRobotState->update();
  m_pHyStateValidator->publishRobotState(*pUngraspedRobotState);

  JointTrajectory fromSupportGroupJntTraj;
  fromSupportGroupJntTraj.resize(traj.size());
  pUngraspedRobotState->copyJointGroupPositions(fromSupportGroup, fromSupportGroupJntTraj[traj.size() - 1]);

  JointTrajectory fromSupportEefGroupJntTraj;
  fromSupportEefGroupJntTraj.resize(traj.size());
  pUngraspedRobotState->copyJointGroupPositions(fromSupportEefGroup, fromSupportEefGroupJntTraj[traj.size() - 1]);

  for (std::size_t i = 0; i < traj.size() - 1; ++i)
  {
    traj[i]->update();
    traj[i]->copyJointGroupPositions(fromSupportGroup, fromSupportGroupJntTraj[i]);
    traj[i]->copyJointGroupPositions(fromSupportEefGroup, fromSupportEefGroupJntTraj[i]);
  }

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

  m_pHyStateValidator->publishRobotState(*pUngraspedRobotState);
  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pUngraspedRobotState->computeCartesianPath(
                                                                         pUngraspedRobotState->getJointModelGroup(fromSupportGroup),
                                                                                                                  traj,
                                                                                                                  pTipLink,
                                                                                                                  toolTipPose,
                                                                                                                  true,
                                                                                                                  0.003,
                                                                                                                  0.0);

  if (!((foundCartesianPath - 1.0) <= std::numeric_limits<double>::epsilon()))
  {
    return false;
  }

  // removable
  pUngraspedRobotState->update();
  m_pHyStateValidator->publishRobotState(*pUngraspedRobotState);

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
