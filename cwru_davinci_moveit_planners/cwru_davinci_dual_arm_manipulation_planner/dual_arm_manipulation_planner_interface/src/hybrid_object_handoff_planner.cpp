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

using namespace dual_arm_manipulation_planner_interface;
namespace ob = ompl::base;
namespace og = ompl::geometric;

HybridObjectHandoffPlanner::HybridObjectHandoffPlanner
(
const ob::State *start,
const ob::State *goal,
const double se3BoundXAxisMin,
const double se3BoundXAxisMax,
const double se3BoundYAxisMin,
const double se3BoundYAxisMax,
const double se3BoundZAxisMin,
const double se3BoundZAxisMax,
const int armIdxLwBd,
const int armIdxUpBd,
const int graspIdxLwBd,
const int graspIdxUpBd,
const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
const robot_model::RobotModelConstPtr& pRobotModel,
const std::string &objectName,
const double maxDistance,
bool verbose
)
: m_Verbose(verbose), m_ObjectName(objectName)
{
  m_pHyStateSpace = std::make_shared<HybridObjectStateSpace>(se3BoundXAxisMin,
                                                             se3BoundXAxisMax,
                                                             se3BoundYAxisMin,
                                                             se3BoundYAxisMax,
                                                             se3BoundZAxisMin,
                                                             se3BoundZAxisMax,
                                                             armIdxLwBd,
                                                             armIdxUpBd,
                                                             graspIdxLwBd,
                                                             graspIdxUpBd,
                                                             possible_grasps);
  if(!m_pHyStateSpace)
  {
    m_pSpaceInfor = nullptr;
    m_pProblemDef = nullptr;
    m_pRRTConnectPlanner = nullptr;
    printf("HybridObjectHandoffPlanner: Failed to initialize hybrid object state space");
  }
  else
  {
    setupSpaceInformation(m_pHyStateSpace, pRobotModel, objectName);
    if(m_pSpaceInfor)
    {
      setupProblemDefinition(start, goal);
      if(m_pProblemDef)
      {
        setupPlanner(maxDistance);
        if(m_Verbose)
        {
          m_pSpaceInfor->printSettings(std::cout);
          m_pProblemDef->print(std::cout);
        }
      }
    }
  }
}

ob::PlannerStatus::StatusType HybridObjectHandoffPlanner::solve
(
const double solveTime
)
{
  if(m_pRRTConnectPlanner && m_pRRTConnectPlanner->isSetup())
  {
    m_Solved = m_pRRTConnectPlanner->ob::Planner::solve(solveTime);
    return m_Solved;
  }

  m_Solved = ob::PlannerStatus::ABORT;
  return m_Solved;
}

bool HybridObjectHandoffPlanner::getSolutionPathJointTrajectory
(
SolutionPathJointTrajectory& wholePathJntTraj
)
{
  if (!m_Solved || !m_pProblemDef->hasExactSolution())
  {
    return false;
  }

  wholePathJntTraj.clear();
  og::PathGeometric *slnPath = m_pProblemDef->getSolutionPath()->as<og::PathGeometric>();
  const std::vector<ob::State*>& constSlnStates = slnPath->getStates();
  const size_t segments = slnPath->getStateCount()-1;

  for(size_t i = 0; i < segments; ++i)
  {
    SolutionPathJointTrajectory jntTrajectoryBtwStates;
    if(!connectStates(constSlnStates[i], constSlnStates[i + 1], jntTrajectoryBtwStates))
    {
      return false;
    }
  }
}

void HybridObjectHandoffPlanner::setupSpaceInformation
(
const HybridObjectStateSpacePtr& pHyStateSpace,
const robot_model::RobotModelConstPtr& pRobotModel,
const std::string &objectName
)
{
  m_pSpaceInfor = std::make_shared<ob::SpaceInformation>(m_pHyStateSpace);
  if(!m_pSpaceInfor)
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
const ompl::base::State *start,
const ompl::base::State *goal
)
{
  if(!m_pSpaceInfor->isValid(start) || !m_pSpaceInfor->isValid(goal))
  {
    m_pRRTConnectPlanner = nullptr;
    printf("HybridObjectHandoffPlanner: Either start or goal state is not valid");
    return;
  }

  m_pProblemDef = std::make_shared<ob::ProblemDefinition>(m_pSpaceInfor);
  if(!m_pProblemDef)
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
  if(!m_pRRTConnectPlanner)
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
  const ompl::base::State *pFromState,
  const ompl::base::State *pToState,
  SolutionPathJointTrajectory &jntTrajectoryBtwStates
  )
{
  const HybridObjectStateSpace::StateType* pHyFromState = dynamic_cast<const HybridObjectStateSpace::StateType*>(pFromState);
  const HybridObjectStateSpace::StateType* pHyToState = dynamic_cast<const HybridObjectStateSpace::StateType*>(pToState);

  if(!pHyFromState || !pHyToState)
  {
    printf("HybridObjectHandoffPlanner: Invalid states to be connected");
    return false;
  }

  bool goodPath = false;
  switch(m_pHyStateSpace->checkStateDiff(pHyFromState, pHyToState))
  {
    case StateDiff::AllSame:
      goodPath = true;
      break;
    case StateDiff::PoseDiffArmAndGraspSame:
    {
      goodPath = planObjectTransit(pHyFromState, pHyFromState, jntTrajectoryBtwStates);
      break;
    }
    case StateDiff::ArmAndGraspDiffPoseSame:
      goodPath = planHandoff(pHyFromState, pHyFromState, jntTrajectoryBtwStates);
      break;
    default:
      // should not be there
      break;
  }
  return goodPath;
}

bool HybridObjectHandoffPlanner::planObjectTransit
(
const HybridObjectStateSpace::StateType* pHyFromState,
const HybridObjectStateSpace::StateType* pHyToState,
SolutionPathJointTrajectory &jntTrajectoryBtwStates
)
{
  if(!m_pHyStateValidator)
  {
    printf("HybridObjectHandoffPlanner: Failed to connect states");
    return false;
  }

  const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  if(!pRobotFromState || !m_pHyStateValidator->hybridStateToRobotState(pHyFromState, pRobotFromState))
  {
    printf("HybridObjectHandoffPlanner: Invalid FromState to be connected");
    return false;
  }
  const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  if(!pRobotToState || !m_pHyStateValidator->hybridStateToRobotState(pHyToState, pRobotToState))
  {
    printf("HybridObjectHandoffPlanner: Invalid ToState to be connected");
    return false;
  }

  const std::string supportGroup = (pHyFromState->armIndex().value == 1) ? "psm_one" : "psm_two";
  const robot_state::JointModelGroup *pSupportJntGroup = pRobotToState->getJointModelGroup(supportGroup);
  const moveit::core::LinkModel *pTipLink = pSupportJntGroup->getOnlyOneEndEffectorTip();
  const Eigen::Affine3d toolTipPose = pRobotToState->getGlobalLinkTransform(pTipLink);

  std::vector<robot_state::RobotStatePtr> traj;
  double foundCartesianPath = pRobotFromState->computeCartesianPath(pRobotFromState->getJointModelGroup(supportGroup),
                                                                    traj,
                                                                    pTipLink,
                                                                    toolTipPose,
                                                                    true,
                                                                    0.001,
                                                                    0.0);

  if (foundCartesianPath != 1.0)
  {
    return false;
  }

  // removable
  m_pHyStateValidator->setMimicJointPositions(pRobotFromState, supportGroup);
  pRobotFromState->update();
  // publishRobotState(*cp_start_state);

  JointTrajectory supportGroupJntTraj;
  supportGroupJntTraj.resize(traj.size());
  pRobotToState->copyJointGroupPositions(supportGroup, supportGroupJntTraj[traj.size() - 1]);

  for (std::size_t i = 0; i < traj.size() - 1; ++i)
  {
    traj[i]->update();
    traj[i]->copyJointGroupPositions(supportGroup, supportGroupJntTraj[i]);
  }

  PlanningGroupJointTrajectory supportGroupObjectTransitJntTraj = {{supportGroup, supportGroupJntTraj}};
  jntTrajectoryBtwStates[0] = supportGroupObjectTransitJntTraj;
  return true;
}

bool HybridObjectHandoffPlanner::planHandoff
(
const HybridObjectStateSpace::StateType* pHyFromState,
const HybridObjectStateSpace::StateType* pHyToState,
SolutionPathJointTrajectory &jntTrajectoryBtwStates
)
{
  if(!m_pHyStateValidator)
  {
    printf("HybridObjectHandoffPlanner: Failed to connect states");
    return false;
  }

  const robot_state::RobotStatePtr pRobotFromState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  if(!pRobotFromState || !m_pHyStateValidator->hybridStateToRobotState(pHyFromState, pRobotFromState))
  {
    printf("HybridObjectHandoffPlanner: Invalid FromState to be connected");
    return false;
  }
  const robot_state::RobotStatePtr pRobotToState(new robot_state::RobotState(m_pHyStateValidator->robotModel()));
  if(!pRobotToState || !m_pHyStateValidator->hybridStateToRobotState(pHyToState, pRobotToState))
  {
    printf("HybridObjectHandoffPlanner: Invalid ToState to be connected");
    return false;
  }

  // an intermediate state when needle is supporting by two grippers
  const robot_state::RobotStatePtr pHandoffRobotState(new robot_state::RobotState(*pRobotFromState));

  const std::string toSupportGroup = (pHyToState->armIndex().value == 1) ? "psm_one" : "psm_two";
  std::vector<double> toSupportGroupJntPosition;
  pRobotToState->copyJointGroupPositions(toSupportGroup, toSupportGroupJntPosition);
  pHandoffRobotState->setJointGroupPositions(toSupportGroup, toSupportGroupJntPosition);
  m_pHyStateValidator->setMimicJointPositions(pHandoffRobotState, toSupportGroup);

  const moveit::core::AttachedBody *ss_needle_body = pRobotFromState->getAttachedBody(m_ObjectName);
  const moveit::core::AttachedBody *gs_needle_body = pRobotToState->getAttachedBody(m_ObjectName);

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


  pHandoffRobotState->attachBody(gs_needle_body->getName(), gs_needle_body->getShapes(),
                            gs_needle_body->getFixedTransforms(), touch_links,
                            gs_needle_body->getAttachedLinkName(), gs_needle_body->getDetachPosture());
  pHandoffRobotState->update();

}