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
: m_verbose(verbose)
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
        if(m_verbose)
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
    m_solved = m_pRRTConnectPlanner->ob::Planner::solve(solveTime);
    return m_solved;
  }

  m_solved = ob::PlannerStatus::ABORT;
  return m_solved;
}

bool HybridObjectHandoffPlanner::getSolutionPathJointTrajectory
(
SolutionPathJointTrajectory& wholePathJntTraj
)
{
  if (!m_solved || !m_pProblemDef->hasExactSolution())
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
const ompl::base::State* fromState,
const ompl::base::State* toState,
SolutionPathJointTrajectory& jntTrajectoryBtwStates
)
{
  bool goodPath = false;
  return goodPath;
}