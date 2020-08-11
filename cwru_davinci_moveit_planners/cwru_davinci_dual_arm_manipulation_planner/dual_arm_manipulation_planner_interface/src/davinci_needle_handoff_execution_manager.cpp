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

#include <dual_arm_manipulation_planner_interface/davinci_needle_handoff_execution_manager.h>
#include <std_srvs/SetBool.h>
#include <uv_msgs/pf_grasp.h>

#define DEFAULT_NEEDLE_POSE_TOPIC "/updated_needle_pose"
#define DEFAULT_JAW_OPENING 0.5

using namespace dual_arm_manipulation_planner_interface;
namespace ob = ompl::base;

DavinciNeedleHandoffExecutionManager::DavinciNeedleHandoffExecutionManager
(
const ros::NodeHandle& nodeHandle,
const ros::NodeHandle& nodeHandlePrivate,
const std::vector<cwru_davinci_grasp::GraspInfo>& possibleGrasps,
const std::string& objectName,
const std::string& robotDescription
)
 : m_NodeHandle(nodeHandle),
   m_NodeHandlePrivate(nodeHandlePrivate),
   m_GraspInfo(possibleGrasps),
   m_ObjectName(objectName),
   m_RobotModelLoader(robotDescription),
   m_NeedlePoseMd(nodeHandle, nodeHandlePrivate),
   m_UniformRealDistribution(-0.2, 0.2),
   m_PiecewiseDistribution(m_Intervals.begin(), m_Intervals.end(), m_Weights.begin())
{
  m_pHandoffPlanner = std::make_shared<HybridObjectHandoffPlanner>();

  if (!nodeHandlePrivate.param<std::string>("needle_pose_topic", m_NEEDLE_POSE_TOPIC, DEFAULT_NEEDLE_POSE_TOPIC))
  {
    m_NeedlePoseSub = 
        m_NodeHandle.subscribe(DEFAULT_NEEDLE_POSE_TOPIC, 1, &DavinciNeedleHandoffExecutionManager::needlePoseCallBack, this);
  }
  else
  {
    m_NeedlePoseSub = 
        m_NodeHandle.subscribe(m_NEEDLE_POSE_TOPIC, 1, &DavinciNeedleHandoffExecutionManager::needlePoseCallBack, this);
  }

  nodeHandlePrivate.param<double>("jaw_opening", m_JawOpening, DEFAULT_JAW_OPENING);
  m_PfGraspClient = m_NodeHandle.serviceClient<uv_msgs::pf_grasp>("/pf_grasp");

  m_PSMOneStickyFingerClient = m_NodeHandle.serviceClient<std_srvs::SetBool>("sticky_finger/PSM1_tool_wrist_sca_ee_link_1");
  m_PSMTwoStickyFingerClient = m_NodeHandle.serviceClient<std_srvs::SetBool>("sticky_finger/PSM2_tool_wrist_sca_ee_link_1");
}

DavinciNeedleHandoffExecutionManager::~DavinciNeedleHandoffExecutionManager
(
)
{
  if (m_pHyStartState && m_pHyGoalState)
  {
    m_pHandoffPlanner->m_pHyStateSpace->freeState(m_pHyStartState);
    m_pHandoffPlanner->m_pHyStateSpace->freeState(m_pHyGoalState);
  }

  if (m_pHyFailedAtState)
    m_pHandoffPlanner->m_pHyStateSpace->freeState(m_pHyFailedAtState);
}

bool DavinciNeedleHandoffExecutionManager::executeNeedleHandoffTraj
(
)
{
  moveit_msgs::MoveItErrorCodes errorCodes;
  if (m_PlanningStatus != ob::PlannerStatus::EXACT_SOLUTION || m_HandoffJntTraj.empty())
  {
    errorCodes.val = errorCodes.FAILURE;
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: "
             "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
    return false;
  }

  int lastHandoffIdx = lastHandoffTrajSeg();
  ROS_INFO("DavinciNeedleHandoffExecutionManager: total number of trajectories is %d", (int)m_HandoffJntTraj.size());
  for (std::size_t i = 0; i < m_HandoffJntTraj.size(); ++i)
  {
    if (m_HandoffJntTraj[i].size() == 1)  // object Transit
    {
      const MoveGroupJointTrajectorySegment& jntTrajSeg = m_HandoffJntTraj[i][0].second;
      m_pSupportArmGroup.reset(new psm_interface(jntTrajSeg.begin()->first, m_NodeHandle));
      if(!correctObjectTransfer(i + 1))
      {
        ROS_ERROR("DavinciNeedleHandoffExecutionManager: Failed to execute needle transfer trajectory");
        return false;
      }
    }
    else if (m_HandoffJntTraj[i].size() == 4)  // object Transfer
    {
      if (!correctObjectTransit(i, m_HandoffJntTraj[i]))
      {
        return false;
      }

      m_pMoveItSupportArmGroupInterf.reset(new MoveGroupInterface(m_HandoffJntTraj[i][2].second.begin()->first));
      m_pMoveItSupportArmGroupInterf->detachObject(m_ObjectName);

      // move in fashion: home to pregrasp, approach-grasp, ungrasp-retreat, back to home
      const MoveGroupJointTrajectorySegment& safePlaceToPreGraspJntTrajSeg = m_HandoffJntTraj[i][0].second;
      m_pSupportArmGroup.reset(new psm_interface(safePlaceToPreGraspJntTrajSeg.begin()->first, m_NodeHandle));
      {
        double jawPosition = 0.0;
        m_pSupportArmGroup->get_gripper_fresh_position(jawPosition);
        const JointTrajectory& jntTra = safePlaceToPreGraspJntTrajSeg.begin()->second;
        if (!m_pSupportArmGroup->execute_trajectory(jntTra, jawPosition, 0.03))
        {
          ROS_ERROR("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
          return false;
        }
      }

      const MoveGroupJointTrajectorySegment& preGraspToGraspedJntTrajSeg = m_HandoffJntTraj[i][1].second;
      m_pSupportArmGroup.reset(new psm_interface(preGraspToGraspedJntTrajSeg.begin()->first, m_NodeHandle));
      // open gripper of incoming supporting arm
      m_pSupportArmGroup->control_jaw(m_JawOpening, 0.2);

      m_pMoveItSupportArmGroupInterf.reset(new MoveGroupInterface(preGraspToGraspedJntTrajSeg.begin()->first));
      turnOnStickyFinger(m_pSupportArmGroup->get_psm_name());
      {
        const JointTrajectory& armJntTra = preGraspToGraspedJntTrajSeg.begin()->second;
        // const JointTrajectory& gripperJntTra = (++preGraspToGraspedJntTrajSeg.begin())->second;
        double jawPosition = m_JawOpening;
        if (!m_pSupportArmGroup->execute_trajectory(armJntTra, jawPosition, 0.05))
        {
          ROS_ERROR("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
          return false;
        }
        ros::Duration(1.0).sleep();
        m_pMoveItSupportArmGroupInterf->attachObject(m_ObjectName);
      }

      // close gripper of incoming supporting arm
      m_pSupportArmGroup->control_jaw(0.0, 0.2);
      const std::string& toSupportGroup = m_pSupportArmGroup->get_psm_name();
      changeNeedleTrackerMode(i + 1);

      const MoveGroupJointTrajectorySegment& graspToUngraspedJntSeg = m_HandoffJntTraj[i][2].second;
      m_pSupportArmGroup.reset(new psm_interface(graspToUngraspedJntSeg.begin()->first, m_NodeHandle));
      // open gripper of incoming resting arm
      turnOffStickyFinger(m_pSupportArmGroup->get_psm_name());
      m_pSupportArmGroup->control_jaw(m_JawOpening, 0.2);
      {
        const JointTrajectory& armJntTra = graspToUngraspedJntSeg.begin()->second;
        // const JointTrajectory& gripperJntTra = (++graspToUngraspedJntSeg.begin())->second;
        double jawPosition = m_JawOpening;
        if (!m_pSupportArmGroup->execute_trajectory(armJntTra, jawPosition, 0.03))
        {
          ROS_ERROR("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
          return false;
        }
      }

      const std::string& toRestGroup = m_pSupportArmGroup->get_psm_name();
      // if needle perturbation happens it is only allowed to happen at non-last trajectory segment
      if ((i != lastHandoffIdx) && !perturbNeedlePose(i, toSupportGroup))
        ROS_INFO("DavinciNeedleHandoffExecutionManager: needle pose is NOT perturbed");
      else
        ROS_INFO("DavinciNeedleHandoffExecutionManager: needle pose is perturbed");

      m_pSupportArmGroup.reset(new psm_interface(toRestGroup, m_NodeHandle));
      // close gripper of incoming reseting arm
      m_pSupportArmGroup->control_jaw(0.0, 0.2);

      const MoveGroupJointTrajectorySegment& ungraspedToSafePlaceJntTrajSeg = m_HandoffJntTraj[i][3].second;
      m_pSupportArmGroup.reset(new psm_interface(ungraspedToSafePlaceJntTrajSeg.begin()->first, m_NodeHandle));
      {
        double jawPosition = 0.0;
        m_pSupportArmGroup->get_gripper_fresh_position(jawPosition);
        const JointTrajectory& jntTra = ungraspedToSafePlaceJntTrajSeg.begin()->second;
        if (!m_pSupportArmGroup->execute_trajectory(jntTra, jawPosition, 0.03))
        {
          ROS_ERROR("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
          return false;
        }
      }
      ROS_INFO("DavinciNeedleHandoffExecutionManager: the number %d trajectory has been executed", (int)i);

      // after handoff motion, correct needle pose once
      m_pSupportArmGroup.reset(new psm_interface(toSupportGroup, m_NodeHandle));
      if(!correctObjectTransfer(i + 1) && (i == m_HandoffJntTraj.size()))
      {
        ROS_ERROR("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
        return false;
      }
    }
  }

  ROS_INFO("DavinciNeedleHandoffExecutionManager: all trajectories have been executed");
  return true;
}

bool DavinciNeedleHandoffExecutionManager::planNeedleHandoffTraj
(
const double solveTime
)
{
  if (m_PlanningStatus == ob::PlannerStatus::UNKNOWN)
  {
    m_PlanningStatus = m_pHandoffPlanner->solve(solveTime);
    if (m_PlanningStatus == ob::PlannerStatus::EXACT_SOLUTION)
    {
      if (m_pHandoffPlanner->getSolutionPathJointTrajectory(m_HandoffJntTraj))
      {
        return true;
      }
      else
      {
        ROS_ERROR("DavinciNeedleHandoffExecutionManager: Failed to generate handoff trajectories");
        return false;
      }
    }
    else
    {
      ROS_ERROR("DavinciNeedleHandoffExecutionManager: Needle handoff planning failed, the error code is %s",
               m_PlanningStatus.asString().c_str());
      m_HandoffJntTraj.resize(0);
      return false;
    }
  }
}

bool DavinciNeedleHandoffExecutionManager::constructStartAndGoalState
(
const ompl::base::SE3StateSpace::StateType* objStartPose,
const int startSupportArmIdx,
const int startGraspIdx,
const std::vector<double>& startJointPosition,
const ompl::base::SE3StateSpace::StateType* objGoalPose,
const int goalSupportArmIdx,
const int goalGraspIdx,
const std::vector<double>& goalJointPosition
)
{
  if (!m_pHandoffPlanner->m_pHyStateSpace)
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: invalid hybrid state space pointer");
    return false;
  }

  m_pHyStartState = m_pHandoffPlanner->m_pHyStateSpace->allocState()->as<HybridObjectStateSpace::StateType>();
  m_pHyGoalState = m_pHandoffPlanner->m_pHyStateSpace->allocState()->as<HybridObjectStateSpace::StateType>();

  //  construct start state
  m_pHandoffPlanner->m_pHyStateSpace->getSubspaces()[0]->copyState(m_pHyStartState->components[0], objStartPose);

  m_pHyStartState->armIndex().value = startSupportArmIdx;
  m_pHyStartState->graspIndex().value = startGraspIdx;
  m_pHandoffPlanner->m_pHyStateSpace->setJointValues(startJointPosition, m_pHyStartState);
  m_pHyStartState->setJointsComputed(true);
  m_pHyStartState->markValid();

  //  construct goal state
  m_pHandoffPlanner->m_pHyStateSpace->getSubspaces()[0]->copyState(m_pHyGoalState->components[0], objGoalPose);

  m_pHyGoalState->armIndex().value = goalSupportArmIdx;
  m_pHyGoalState->graspIndex().value = goalGraspIdx;
  m_pHandoffPlanner->m_pHyStateSpace->setJointValues(goalJointPosition, m_pHyGoalState);
  m_pHyGoalState->setJointsComputed(true);
  m_pHyGoalState->markValid();

  return true;
}

bool DavinciNeedleHandoffExecutionManager::setupStartAndGoalStateInPlanner
(
const ompl::base::ScopedState<HybridObjectStateSpace>& start,
const ompl::base::ScopedState<HybridObjectStateSpace>& goal
)
{
  if (!m_pHandoffPlanner)
  {
    return false;
  }
  if (!m_pHandoffPlanner->m_pSpaceInfor->satisfiesBounds(start.get())
      || !m_pHandoffPlanner->m_pSpaceInfor->satisfiesBounds(goal.get()))
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: Either start or goal state is NOT within the bounds");
    return false;
  }

  m_pHandoffPlanner->m_pRRTConnectPlanner->clear();
  m_pHandoffPlanner->setupProblemDefinition(start.get(), goal.get());
  m_PlanningStatus = ompl::base::PlannerStatus::UNKNOWN;
  m_pHandoffPlanner->m_pRRTConnectPlanner->setProblemDefinition(m_pHandoffPlanner->m_pProblemDef);

  if (!m_pHandoffPlanner->m_pRRTConnectPlanner->isSetup())
  {
    m_pHandoffPlanner->m_pRRTConnectPlanner->setup();
  }
  return true;
}

bool DavinciNeedleHandoffExecutionManager::initializePlanner
(
bool withStartAndGoalState
)
{
  if (!m_NodeHandlePrivate.hasParam("se3_bounds"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `se3_bounds` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << m_NodeHandlePrivate.getNamespace());
    return false;
  }

  XmlRpc::XmlRpcValue xmlSE3BoundsArray;
  m_NodeHandlePrivate.getParam("se3_bounds", xmlSE3BoundsArray);
  if (xmlSE3BoundsArray.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (std::size_t i = 0; i < xmlSE3BoundsArray.size(); ++i)
    {
      ROS_ASSERT(xmlSE3BoundsArray[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      m_SE3Bounds[i] = static_cast<double>(xmlSE3BoundsArray[i]);
    }
  }
  else
  {
    ROS_ERROR_STREAM("SE3 bounds type is not type array?");
  }

  if (!m_NodeHandlePrivate.hasParam("arm_index_bounds"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `arm_index_bounds` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << m_NodeHandlePrivate.getNamespace());
    return false;
  }

  XmlRpc::XmlRpcValue xmlArmIndexBounds;
  m_NodeHandlePrivate.getParam("arm_index_bounds", xmlArmIndexBounds);
  if (xmlArmIndexBounds.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (std::size_t i = 0; i < xmlArmIndexBounds.size(); ++i)
    {
      ROS_ASSERT(xmlArmIndexBounds[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
      m_ArmIndexBounds[i] = static_cast<int>(xmlArmIndexBounds[i]);
    }
  }
  else
  {
    ROS_ERROR_STREAM("ArmIndex bounds type is not type array?");
  }

  if (!m_NodeHandlePrivate.hasParam("object_name"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `object_name` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                      << m_NodeHandlePrivate.getNamespace());
    return false;
  }
  std::string objectName;
  m_NodeHandlePrivate.getParam("object_name", objectName);

  if (!m_NodeHandlePrivate.hasParam("max_distance"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `max_distance` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << m_NodeHandlePrivate.getNamespace());
    return false;
  }
  double maxDistance;
  m_NodeHandlePrivate.getParam("max_distance", maxDistance);

  if (m_GraspInfo.empty())
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: input grasp info list is empty");
    return false;
  }

  if (!m_pHandoffPlanner)
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: invalid handoff planner pointer");
    return false;
  }

  m_pHandoffPlanner->setupStateSpace(m_GraspInfo);

  if (!m_pHandoffPlanner->m_pHyStateSpace)
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: failed to setup up hybrid object state space");
    return false;
  }

  m_pHandoffPlanner->m_pHyStateSpace->setSE3Bounds(m_SE3Bounds[0],
                                                   m_SE3Bounds[1],
                                                   m_SE3Bounds[2],
                                                   m_SE3Bounds[3],
                                                   m_SE3Bounds[4],
                                                   m_SE3Bounds[5]);

  m_pHandoffPlanner->m_pHyStateSpace->setArmIndexBounds(m_ArmIndexBounds[0],
                                                        m_ArmIndexBounds[1]);

  m_pHandoffPlanner->m_pHyStateSpace->setGraspIndexBounds(0, m_GraspInfo.size()-1);

  m_pHandoffPlanner->setupSpaceInformation(m_pHandoffPlanner->m_pHyStateSpace,
                                           m_RobotModelLoader.getModel(),
                                           objectName);

  if (withStartAndGoalState)
  {
    m_pHandoffPlanner->setupProblemDefinition(m_pHyStartState, m_pHyGoalState);
    m_pHandoffPlanner->m_pHyStateSpace->enforceBounds(m_pHyGoalState);
  }

  m_PlanningStatus = ompl::base::PlannerStatus::UNKNOWN;
  m_pHandoffPlanner->setupPlanner(maxDistance);
  return true;
}

const Eigen::Affine3d& DavinciNeedleHandoffExecutionManager::updateNeedlePose
(
)
{
  ros::Duration(2.0).sleep();
  while (!m_FreshNeedlePose)
  {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  m_FreshNeedlePose = false;

  return m_NeedlePose;
}

bool DavinciNeedleHandoffExecutionManager::turnOnStickyFinger
(
const std::string& supportArmGroup
)
{
  std_srvs::SetBool graspCommand;
  graspCommand.request.data = true;
  (supportArmGroup == "psm_one") ? m_PSMOneStickyFingerClient.call(graspCommand) : m_PSMTwoStickyFingerClient.call(graspCommand);
}

bool DavinciNeedleHandoffExecutionManager::turnOffStickyFinger
(
const std::string& supportArmGroup
)
{
  std_srvs::SetBool graspCommand;
  graspCommand.request.data = false;
  (supportArmGroup == "psm_one") ? m_PSMOneStickyFingerClient.call(graspCommand) : m_PSMTwoStickyFingerClient.call(graspCommand);
}

void DavinciNeedleHandoffExecutionManager::needlePoseCallBack
(
const geometry_msgs::PoseStamped& needlePose
)
{
  m_FreshNeedlePose = true;
  m_NeedlePose.translation().x() = needlePose.pose.position.x;
  m_NeedlePose.translation().y() = needlePose.pose.position.y;
  m_NeedlePose.translation().z() = needlePose.pose.position.z;

  tf::poseMsgToEigen(needlePose.pose, m_NeedlePose);
}

bool DavinciNeedleHandoffExecutionManager::changeNeedleTrackerMode
(
int ithState
)
{
  const HybridObjectStateSpace::StateType* pHyState = 
    m_pHandoffPlanner->m_pSlnPath->getState(ithState)->as<HybridObjectStateSpace::StateType>();

  if (!pHyState)
  {
    return false;
  }

  uv_msgs::pf_grasp pfGraspSrv;
  pfGraspSrv.request.psm = m_pSupportArmGroup->get_psm();
  tf::transformEigenToMsg(m_GraspInfo[pHyState->graspIndex().value].grasp_pose, pfGraspSrv.request.grasp_transform);

  if(!m_PfGraspClient.call(pfGraspSrv))
  {
    ROS_WARN("Failed to call pf_grasp service.");
    ros::spinOnce();
  }

  return true;
}

bool DavinciNeedleHandoffExecutionManager::correctObjectTransfer
(
const int targetState
)
{
  if (!m_pSupportArmGroup || !m_pHandoffPlanner)
  {
    return false;
  }

  Eigen::Affine3d currentNeedlePose = updateNeedlePose();

  const HybridObjectStateSpace::StateType* pTargetHyState = m_pHandoffPlanner->m_pSlnPath->getState(targetState)->as<HybridObjectStateSpace::StateType>();

  if (!pTargetHyState)
  {
    return false;
  }

  Eigen::Affine3d targetNeedlePose;
  m_pHandoffPlanner->m_pHyStateSpace->se3ToEigen3d(pTargetHyState, targetNeedlePose);

  const std::string supportGroup = m_pSupportArmGroup->get_psm_name();
  std::vector<double> currentJointPosition;

  while (!currentNeedlePose.isApprox(targetNeedlePose, 2e-3))
  {
    MoveGroupJointTrajectorySegment jntTrajSeg;
    double time = 0.0;
    m_pSupportArmGroup->get_fresh_position(currentJointPosition);

    bool moveForward = m_pHandoffPlanner->localPlanObjectTransfer(currentNeedlePose,
                                                                  targetNeedlePose,
                                                                  currentJointPosition,
                                                                  supportGroup,
                                                                  jntTrajSeg,
                                                                  time);
    if (!moveForward && targetState != (m_HandoffJntTraj.size()+1))
    {
      ROS_WARN("DavinciNeedleHandoffExecutionManager: Needle transfer partial correction failed, keep executing the rest of trajectories");
      return true;  // if either kinematics or collision results in failure, but not bring to final goal state
    }
    else if (!moveForward && targetState == (m_HandoffJntTraj.size()+1))
    {
      ROS_ERROR("DavinciNeedleHandoffExecutionManager: Needle transfer partial correction failed, stops due to no more further step");
      fillFailedState(pTargetHyState->armIndex().value, pTargetHyState->graspIndex().value, currentNeedlePose, currentJointPosition);
      return false;
    }

    double jawPosition = 0.0;
    m_pSupportArmGroup->get_gripper_fresh_position(jawPosition);
    const JointTrajectory& jntTra = jntTrajSeg.begin()->second;
    if (!m_pSupportArmGroup->execute_trajectory(jntTra, jawPosition, 0.03))
    {
      ROS_ERROR("DavinciNeedleHandoffExecutionManager: Failed to execute partial corrected needle transfer trajectory");
      return false;
    }
    currentNeedlePose = updateNeedlePose();
  }

  ROS_INFO("DavinciNeedleHandoffExecutionManager: Needle transfer correction succeeded.");
  return true;
}

bool DavinciNeedleHandoffExecutionManager::correctObjectTransit
(
const int ithTraj,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  if(!m_pHandoffPlanner)
  {
    return false;
  }
  // Take in and compare the snapshot of the needle with planned needle pose
  Eigen::Affine3d desNeedlePose;
  m_pHandoffPlanner->m_pHyStateSpace->se3ToEigen3d(
    m_pHandoffPlanner->m_pSlnPath->getState(ithTraj + 1)->as<HybridObjectStateSpace::StateType>(), desNeedlePose);

  const HybridObjectStateSpace::StateType* currentHyState = m_pHandoffPlanner->m_pSlnPath->getState(ithTraj)->as<HybridObjectStateSpace::StateType>();

  m_pSupportArmGroup.reset(new psm_interface(currentHyState->armIndex().value, m_NodeHandle));
  if(!m_pSupportArmGroup)
  {
    return false;
  }
  std::vector<double> currentJointPosition;
  m_pSupportArmGroup->get_fresh_position(currentJointPosition);

  const Eigen::Affine3d& currentNeedlePose = updateNeedlePose();
  if (currentNeedlePose.isApprox(desNeedlePose, 1e-3) && (distanceBtwTwoRobotStates(currentHyState, currentJointPosition) < 1e-2))
  {
    ROS_INFO("DavinciNeedleHandoffExecutionManager: No need to correct handoff path, use original plan");
    return true;
  }

  MoveGroupJointTrajectory temp = jntTrajectoryBtwStates;  // make a copy in case localPlanObjectTransit fails
  if(!m_pHandoffPlanner->localPlanObjectTransit(currentJointPosition, currentNeedlePose, ithTraj, jntTrajectoryBtwStates))
  {
    if (!m_pHandoffPlanner->validateOriginalHandoffPath(currentNeedlePose, currentJointPosition, temp))
    {
      ROS_ERROR("DavinciNeedleHandoffExecutionManager: Handoff correction failed, Original plan also fails");
      fillFailedState(currentHyState->armIndex().value, currentHyState->graspIndex().value, currentNeedlePose, currentJointPosition);
      return false;
    }
    ROS_WARN("DavinciNeedleHandoffExecutionManager: Handoff correction failed, Original plan looks good");
    jntTrajectoryBtwStates = temp;
    return true;
  }

  ROS_INFO("DavinciNeedleHandoffExecutionManager: Handoff correction succeeded, start to use new plan");
  return true;
}

bool DavinciNeedleHandoffExecutionManager::perturbNeedlePose
(
int ithTrajSeg,
const std::string& toSupportGroup
)
{
  if (!m_pHandoffPlanner)
  {
    return false;
  }

  // const Eigen::Affine3d currentNeedlePose = updateNeedlePose();  // before opening jaw, remember current needle pose
  Eigen::Affine3d currentNeedlePose;
  m_NeedlePoseMd.getCurrentNeedlePose(currentNeedlePose);
  m_pSupportArmGroup.reset(new psm_interface(toSupportGroup, m_NodeHandle));
  m_pSupportArmGroup->control_jaw(0.5, 0.1);
  turnOffStickyFinger(m_pSupportArmGroup->get_psm_name());

  Eigen::Affine3d idealNeedlePose;
  const HybridObjectStateSpace::StateType* pNextState = m_pHandoffPlanner->m_pSlnPath->getState(ithTrajSeg + 1)->as<HybridObjectStateSpace::StateType>();
  m_pHandoffPlanner->m_pHyStateSpace->se3ToEigen3d(pNextState, idealNeedlePose);

  if (!currentNeedlePose.isApprox(idealNeedlePose, 1e-3))
  {
    idealNeedlePose = currentNeedlePose;
  }

  double radToPerturb = m_PiecewiseDistribution(m_RandSeed);
  if (!m_NeedlePoseMd.perturbNeedlePose(radToPerturb, m_GraspInfo[pNextState->graspIndex().value], idealNeedlePose, true))
    return false;
  ROS_INFO("DavinciNeedleHandoffExecutionManager: Defined perturbation radian is %f", radToPerturb);

  turnOnStickyFinger(m_pSupportArmGroup->get_psm_name());
  m_pSupportArmGroup->control_jaw(0.0, 0.05);
  m_NeedlePoseMd.radianOfChange(radToPerturb, idealNeedlePose, updateNeedlePose());
  ROS_INFO("DavinciNeedleHandoffExecutionManager: Actual perturbation radian is %f", radToPerturb);
  return true;
}

int DavinciNeedleHandoffExecutionManager::lastHandoffTrajSeg
(
)
{
  for (std::size_t i = m_HandoffJntTraj.size(); i --> 0;)
  {
    if( m_HandoffJntTraj[i].size() == 4 )
      return i;
  }

  return 0;
}

double DavinciNeedleHandoffExecutionManager::distanceBtwTwoRobotStates
(
const HybridObjectStateSpace::StateType* currentHyState,
const std::vector<double>& currentJointPosition
)
{
  std::vector<double> idealJointPosition;
  m_pHandoffPlanner->m_pHyStateSpace->copyJointValues(currentHyState, idealJointPosition);

  double diffSum = 0.0;
  for (std::size_t i = 0; i < currentJointPosition.size(); ++i)
  {
    diffSum += fabs(currentJointPosition[i] - idealJointPosition[i]);
  }

  return diffSum;
}

bool DavinciNeedleHandoffExecutionManager::globalReplanning
(
const double solveTime
)
{
  // reset start state to latest state
  if (!m_pHyFailedAtState)
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: FailedAtState is not initialized");
    return false;
  }

  ompl::base::ScopedState<HybridObjectStateSpace> newStartState(m_pHandoffPlanner->m_pSpaceInfor);
  newStartState = m_pHyFailedAtState;

  ompl::base::ScopedState<HybridObjectStateSpace> goalState(m_pHandoffPlanner->m_pSpaceInfor);
  goalState = m_pHyGoalState;

  if (!setupStartAndGoalStateInPlanner(newStartState, goalState))
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: Replanning setup fails");
    return false;
  }

  if(!planNeedleHandoffTraj(solveTime))
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: Replanning planning stage fails");
    return false;
  }

  return true;
}

void DavinciNeedleHandoffExecutionManager::fillFailedState
(
int curArmIdx,
int curGraspIdx,
const Eigen::Affine3d& curNeedlePose,
const std::vector<double>& curJointPosition
)
{
  m_pHyFailedAtState = m_pHandoffPlanner->m_pHyStateSpace->allocState()->as<HybridObjectStateSpace::StateType>();

  m_pHandoffPlanner->m_pHyStateSpace->eigen3dToSE3(curNeedlePose, m_pHyFailedAtState);
  m_pHyFailedAtState->armIndex().value = curArmIdx;
  m_pHyFailedAtState->graspIndex().value = curGraspIdx;
  m_pHandoffPlanner->m_pHyStateSpace->setJointValues(curJointPosition, m_pHyFailedAtState);
  m_pHyFailedAtState->setJointsComputed(true);
  m_pHyFailedAtState->markValid();
}