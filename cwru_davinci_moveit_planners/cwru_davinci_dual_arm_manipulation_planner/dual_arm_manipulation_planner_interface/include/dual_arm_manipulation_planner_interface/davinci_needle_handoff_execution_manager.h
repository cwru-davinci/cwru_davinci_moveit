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
 * Description: The class to execute the joint trajectory of needle handoff on daVinci surgical robot
 */

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DAVINCI_NEEDLE_HANDOFF_EXECUTION_MANAGER_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DAVINCI_NEEDLE_HANDOFF_EXECUTION_MANAGER_H

#include <dual_arm_manipulation_planner_interface/hybrid_object_handoff_planner.h>

//ompl
#include <ompl/geometric/PathGeometric.h>

// cwru_davinci_grasp
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>

namespace dual_arm_manipulation_planner_interface
{
class DavinciNeedleHandoffExecutionManager
{
public:
  DavinciNeedleHandoffExecutionManager(){}

  DavinciNeedleHandoffExecutionManager
  (
  const ros::NodeHandle& nodeHandle,
  const ros::NodeHandle& nodeHandlePrivate,
  const std::vector<cwru_davinci_grasp::GraspInfo>& possibleGrasps,
  const std::string& objectName,
  const std::string& robotDescription = "robot_description"
  );

  ~DavinciNeedleHandoffExecutionManager(){};

  bool planNeedleHandoffTraj
  (
  const double solveTime
  );

  bool executeNeedleHandoffTraj();

  bool constructStartAndGoalState
  (
  const ompl::base::SE3StateSpace::StateType* objStartPose,
  const int startSupportArmIdx,
  const int startGraspIdx,
  const std::vector<double>& startJointPosition,
  const ompl::base::SE3StateSpace::StateType* objGoalPose,
  const int goalSupportArmIdx,
  const int goalGraspIdx,
  const std::vector<double>& goalJointPosition
  );

  bool initializePlanner
  (
  );

private:
  typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;

protected:
  std::vector<cwru_davinci_grasp::GraspInfo>                      m_GraspInfo;

  HybridObjectHandoffPlannerPtr                                   m_pHandoffPlanner = nullptr;

  ompl::base::PlannerStatus                                       m_PlanningStatus = ompl::base::PlannerStatus::UNKNOWN;

  PSMInterfacePtr                                                 m_pSupportArmGroup = nullptr;

  std::unique_ptr<MoveGroupInterface>                             m_pMoveItSupportArmGroupInterf = nullptr;

  ros::Subscriber                                                 m_NeedlePoseSub;

  PathJointTrajectory                                             m_HandoffJntTraj;

  ros::NodeHandle                                                 m_NodeHandlePrivate;
  ros::NodeHandle                                                 m_NodeHandle;

  HybridObjectStateSpace::StateType*                              m_pHyStartState = nullptr;
  HybridObjectStateSpace::StateType*                              m_pHyGoalState  = nullptr;

  double                                                          m_SE3Bounds[6];
  int                                                             m_ArmIndexBounds[2];
  std::string                                                     m_ObjectName;
  robot_model_loader::RobotModelLoader                            m_RobotModelLoader;

private:
  bool turnOnStickyFinger
  (
  const std::string& supportArmGroup
  );

  bool turnOffStickyFinger
  (
  const std::string& supportArmGroup
  );
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DAVINCI_NEEDLE_HANDOFF_EXECUTION_MANAGER_H
