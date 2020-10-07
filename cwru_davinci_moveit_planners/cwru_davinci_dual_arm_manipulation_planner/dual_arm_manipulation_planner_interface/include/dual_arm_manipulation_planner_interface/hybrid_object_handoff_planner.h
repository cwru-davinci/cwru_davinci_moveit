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

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_HANDOFF_PLANNER_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_HANDOFF_PLANNER_H

#include <dual_arm_manipulation_planner_interface/hybrid_motion_validator.h>

// ompl
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <cwru_davinci/uv_control/psm_interface.h>

namespace dual_arm_manipulation_planner_interface
{

enum class TrajectoryType
{
  ObjectTransit,
  SafePlaceToPreGrasp,
  PreGraspToGrasped,
  GraspedToUngrasped,
  UngrasedToSafePlace
};

typedef std::vector<double> JointTrajectoryPoint;
typedef std::vector<JointTrajectoryPoint> JointTrajectory;
typedef std::string MoveGroup;
typedef std::map<MoveGroup, JointTrajectory>  MoveGroupJointTrajectorySegment;
typedef std::vector<std::pair<TrajectoryType, MoveGroupJointTrajectorySegment>> MoveGroupJointTrajectory;
typedef std::vector<MoveGroupJointTrajectory> PathJointTrajectory;

class HybridObjectHandoffPlanner
{
public:
  HybridObjectHandoffPlanner
  (
  bool verbose = true
  );

  HybridObjectHandoffPlanner
  (
  const std::vector<cwru_davinci_grasp::GraspInfo>& graspInfo,
  bool verbose = true
  );

  ~HybridObjectHandoffPlanner(){}

  ompl::base::PlannerStatus::StatusType solve
  (
  const double solveTime
  );

  bool getSolutionPathJointTrajectory
  (
  PathJointTrajectory& handoffPathJntTraj
  );

protected:
  HybridObjectStateSpacePtr                        m_pHyStateSpace = nullptr;

  std::shared_ptr<HybridStateValidityChecker>      m_pHyStateValidator = nullptr;

  ompl::base::SpaceInformationPtr                  m_pSpaceInfor = nullptr;

  ompl::base::ProblemDefinitionPtr                 m_pProblemDef = nullptr;

  std::shared_ptr<ompl::geometric::RRTConnect>     m_pRRTConnectPlanner;

  ompl::base::PlannerStatus                        m_Solved;

  bool                                             m_Verbose;

  std::shared_ptr<ompl::geometric::PathGeometric>  m_pSlnPath = nullptr;

protected:
  void setupStateSpace
  (
  const std::vector<cwru_davinci_grasp::GraspInfo>& graspInfo
  );

  void setupSpaceInformation
  (
  const HybridObjectStateSpacePtr& pHyStateSpace,
  const robot_model::RobotModelConstPtr& pRobotModel,
  const std::string& objectName
  );

  void setupProblemDefinition
  (
  const ompl::base::State* start,
  const ompl::base::State* goal
  );

  void setupPlanner
  (
  const double maxDistance
  );

  bool connectStates
  (
  const ompl::base::State* pFromState,
  const ompl::base::State* pToState,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

  bool planObjectTransit
  (
  const HybridObjectStateSpace::StateType* pHyFromState,
  const HybridObjectStateSpace::StateType* pHyToState,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

  bool planHandoff
  (
  const HybridObjectStateSpace::StateType* pHyFromState,
  const HybridObjectStateSpace::StateType* pHyToState,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

  bool planNeedleGrasping
  (
  const robot_state::RobotStatePtr& pRobotFromState,
  const robot_state::RobotStateConstPtr& pHandoffRobotState,
  const std::string& toSupportGroup,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

  bool planPreGraspStateToGraspedState
  (
  robot_state::RobotStatePtr& pPreGraspRobotState,
  const robot_state::RobotStateConstPtr& pHandoffRobotState,
  const std::string& toSupportGroup,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

  bool planSafeStateToPreGraspState
  (
  const robot_state::RobotStatePtr& pRobotFromState,
  const robot_state::RobotStateConstPtr& pPreGraspRobotState,
  const std::string& toSupportGroup,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

  bool planNeedleReleasing
  (
  const robot_state::RobotStateConstPtr& pHandoffRobotState,
  const robot_state::RobotStateConstPtr& pRobotToState,
  const std::string& fromSupportGroup,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

  bool planGraspStateToUngraspedState
  (
  const robot_state::RobotStateConstPtr& pHandoffRobotState,
  robot_state::RobotStatePtr& pUngraspedRobotState,
  const std::string& fromSupportGroup,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

  bool planUngraspedStateToSafeState
  (
  const robot_state::RobotStatePtr& pUngraspedRobotState,
  const robot_state::RobotStateConstPtr& pRobotToState,
  const std::string& fromSupportGroup,
  MoveGroupJointTrajectory& jntTrajectoryBtwStates
  );

protected:
  friend class DavinciNeedleHandoffExecutionManager;
};

typedef std::shared_ptr<HybridObjectHandoffPlanner> HybridObjectHandoffPlannerPtr;

}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_HANDOFF_PLANNER_H
