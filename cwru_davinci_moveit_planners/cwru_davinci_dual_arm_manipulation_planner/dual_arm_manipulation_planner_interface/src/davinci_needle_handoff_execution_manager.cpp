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
   m_RobotModelLoader(robotDescription)
{
  m_pHandoffPlanner = std::make_shared<HybridObjectHandoffPlanner>();

  m_NeedlePoseSub =
    m_NodeHandle.subscribe("/updated_needle_pose", 1, &DavinciNeedleHandoffExecutionManager::needlePoseCallBack, this);

  m_PfGraspClient = m_NodeHandle.serviceClient<uv_msgs::pf_grasp>("/pf_grasp");
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
}

bool DavinciNeedleHandoffExecutionManager::executeNeedleHandoffTraj
(
)
{
  moveit_msgs::MoveItErrorCodes errorCodes;
  if (!m_PlanningStatus == ob::PlannerStatus::EXACT_SOLUTION || m_HandoffJntTraj.empty())
  {
    errorCodes.val = errorCodes.FAILURE;
    ROS_INFO("DavinciNeedleHandoffExecutionManager: "
             "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
    return false;
  }

  ROS_INFO("DavinciNeedleHandoffExecutionManager: total number of trajectories is %d", (int)m_HandoffJntTraj.size());
  for (std::size_t i = 0; i < m_HandoffJntTraj.size(); ++i)
  {
    if (m_HandoffJntTraj[i].size() == 1)  // object Transit
    {
      const MoveGroupJointTrajectorySegment& jntTrajSeg = m_HandoffJntTraj[i][0].second;
      m_pSupportArmGroup.reset(new psm_interface(jntTrajSeg.begin()->first, m_NodeHandle));
      return correctObjectTransfer(i + 1);
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
        if (!m_pSupportArmGroup->execute_trajectory_t(jntTra, jawPosition, 8.0))
        {
          ROS_INFO("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
          return false;
        }
      }

      const MoveGroupJointTrajectorySegment& preGraspToGraspedJntTrajSeg = m_HandoffJntTraj[i][1].second;
      m_pSupportArmGroup.reset(new psm_interface(preGraspToGraspedJntTrajSeg.begin()->first, m_NodeHandle));
      m_pMoveItSupportArmGroupInterf.reset(new MoveGroupInterface(preGraspToGraspedJntTrajSeg.begin()->first));
      turnOnStickyFinger(m_pSupportArmGroup->get_psm_name());
      {
        const JointTrajectory& armJntTra = preGraspToGraspedJntTrajSeg.begin()->second;
        const JointTrajectory& gripperJntTra = (++preGraspToGraspedJntTrajSeg.begin())->second;
        if (!m_pSupportArmGroup->execute_trajectory_t(armJntTra, gripperJntTra, 3.0))
        {
          ROS_INFO("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
          return false;
        }
        ros::Duration(1.0).sleep();
        m_pMoveItSupportArmGroupInterf->attachObject(m_ObjectName);
      }

      changeNeedleTrackerMode(i + 1);

      const MoveGroupJointTrajectorySegment& graspToUngraspedJntSeg = m_HandoffJntTraj[i][2].second;
      m_pSupportArmGroup.reset(new psm_interface(graspToUngraspedJntSeg.begin()->first, m_NodeHandle));
      turnOffStickyFinger(m_pSupportArmGroup->get_psm_name());
      {
        const JointTrajectory& armJntTra = graspToUngraspedJntSeg.begin()->second;
        const JointTrajectory& gripperJntTra = (++graspToUngraspedJntSeg.begin())->second;
        if (!m_pSupportArmGroup->execute_trajectory_t(armJntTra, gripperJntTra, 3.0))
        {
          ROS_INFO("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
          return false;
        }
      }

      const MoveGroupJointTrajectorySegment& ungraspedToSafePlaceJntTrajSeg = m_HandoffJntTraj[i][3].second;
      m_pSupportArmGroup.reset(new psm_interface(ungraspedToSafePlaceJntTrajSeg.begin()->first, m_NodeHandle));
      {
        double jawPosition = 0.0;
        m_pSupportArmGroup->get_gripper_fresh_position(jawPosition);
        const JointTrajectory& jntTra = ungraspedToSafePlaceJntTrajSeg.begin()->second;
        if (!m_pSupportArmGroup->execute_trajectory_t(jntTra, jawPosition, 8.0))
        {
          ROS_INFO("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
          return false;
        }
      }
      ROS_INFO("DavinciNeedleHandoffExecutionManager: the number %d trajectory has been executed", (int)i);
    }
  }

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
        ROS_INFO("DavinciNeedleHandoffExecutionManager: Failed to generate handoff trajectories");
        return false;
      }
    }
    else
    {
      ROS_INFO("DavinciNeedleHandoffExecutionManager: Needle handoff planning failed, the error code is %s",
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
  m_pHyStartState->se3State().setXYZ(objStartPose->getX(),
                                     objStartPose->getY(),
                                     objStartPose->getZ());
  m_pHyStartState->se3State().rotation().x = objStartPose->rotation().x;
  m_pHyStartState->se3State().rotation().y = objStartPose->rotation().y;
  m_pHyStartState->se3State().rotation().z = objStartPose->rotation().z;
  m_pHyStartState->se3State().rotation().w = objStartPose->rotation().w;

  m_pHyStartState->armIndex().value = startSupportArmIdx;
  m_pHyStartState->graspIndex().value = startGraspIdx;
  m_pHandoffPlanner->m_pHyStateSpace->setJointValues(startJointPosition, m_pHyStartState);
  m_pHyStartState->setJointsComputed(true);
  m_pHyStartState->markValid();

  //  construct goal state
  m_pHyGoalState->se3State().setXYZ(objGoalPose->getX(),
                                    objGoalPose->getY(),
                                    objGoalPose->getZ());
  m_pHyGoalState->se3State().rotation().x = objGoalPose->rotation().x;
  m_pHyGoalState->se3State().rotation().y = objGoalPose->rotation().y;
  m_pHyGoalState->se3State().rotation().z = objGoalPose->rotation().z;
  m_pHyGoalState->se3State().rotation().w = objGoalPose->rotation().w;

  m_pHyGoalState->armIndex().value = goalSupportArmIdx;
  m_pHyGoalState->graspIndex().value = goalGraspIdx;
  m_pHandoffPlanner->m_pHyStateSpace->setJointValues(goalJointPosition, m_pHyGoalState);
  m_pHyGoalState->setJointsComputed(true);
  m_pHyGoalState->markValid();

  return true;
}

bool DavinciNeedleHandoffExecutionManager::initializePlanner
(
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

  m_pHandoffPlanner->m_pHyStateSpace->enforceBounds(m_pHyGoalState);
  m_pHandoffPlanner->setupProblemDefinition(m_pHyStartState, m_pHyGoalState);
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
  ros::ServiceClient stickyFingerClient;
  (supportArmGroup == "psm_one") ?
  stickyFingerClient = m_NodeHandle.serviceClient<std_srvs::SetBool>("sticky_finger/PSM1_tool_wrist_sca_ee_link_1") :
  stickyFingerClient = m_NodeHandle.serviceClient<std_srvs::SetBool>("sticky_finger/PSM2_tool_wrist_sca_ee_link_1");

  std_srvs::SetBool graspCommand;
  graspCommand.request.data = true;
  stickyFingerClient.call(graspCommand);
}

bool DavinciNeedleHandoffExecutionManager::turnOffStickyFinger
(
const std::string& supportArmGroup
)
{
  ros::ServiceClient stickyFingerClient;
  (supportArmGroup == "psm_one") ?
  stickyFingerClient = m_NodeHandle.serviceClient<std_srvs::SetBool>("sticky_finger/PSM1_tool_wrist_sca_ee_link_1") :
  stickyFingerClient = m_NodeHandle.serviceClient<std_srvs::SetBool>("sticky_finger/PSM2_tool_wrist_sca_ee_link_1");

  std_srvs::SetBool graspCommand;
  graspCommand.request.data = false;
  stickyFingerClient.call(graspCommand);
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
  const HybridObjectStateSpace::StateType* pTargetHyState = m_pHandoffPlanner->m_pSlnPath->getState(targetState)->as<HybridObjectStateSpace::StateType>();

  if (!pTargetHyState)
  {
    return false;
  }

  Eigen::Affine3d targetNeedlePose;
  m_pHandoffPlanner->m_pHyStateSpace->se3ToEigen3d(pTargetHyState, targetNeedlePose);
  Eigen::Affine3d currentNeedlePose = updateNeedlePose();

  const robot_state::RobotStatePtr pTargetRobotState(new robot_state::RobotState(m_pHandoffPlanner->m_pHyStateValidator->robotModel()));
  const std::string& supportGroup = m_pSupportArmGroup->get_psm_name();
  pTargetRobotState->setJointGroupPositions(supportGroup, pTargetHyState->jointVariables().values);
  pTargetRobotState->update();
  const Eigen::Affine3d targetTipPose = 
    pTargetRobotState->getGlobalLinkTransform(pTargetRobotState->getJointModelGroup(supportGroup)->getOnlyOneEndEffectorTip());

  while (!currentNeedlePose.isApprox(targetNeedlePose, 1e-2))
  {
    MoveGroupJointTrajectorySegment jntTrajSeg;
    double time = 0.0;
    bool moveForward = m_pHandoffPlanner->localPlanObjectTransfer(currentNeedlePose,
                                                                  targetTipPose,
                                                                  supportGroup,
                                                                  m_pSupportArmGroup,
                                                                  jntTrajSeg,
                                                                  time);
    if (!moveForward)
    {
      return false;
    }

    double jawPosition = 0.0;
    m_pSupportArmGroup->get_gripper_fresh_position(jawPosition);
    const JointTrajectory& jntTra = jntTrajSeg.begin()->second;
    if (!m_pSupportArmGroup->execute_trajectory_t(jntTra, jawPosition, time))
    {
      ROS_INFO("DavinciNeedleHandoffExecutionManager: Failed to execute handoff trajectories");
      return false;
    }
    currentNeedlePose = updateNeedlePose();
  }

  return true;
}

bool DavinciNeedleHandoffExecutionManager::correctObjectTransit
(
const int ithTraj,
MoveGroupJointTrajectory& jntTrajectoryBtwStates
)
{
  //Take in and compare the snapshot of the needle with planned needle pose
  Eigen::Affine3d desNeedlePose;
  m_pHandoffPlanner->m_pHyStateSpace->se3ToEigen3d(
    m_pHandoffPlanner->m_pSlnPath->getState(ithTraj + 1)->as<HybridObjectStateSpace::StateType>(), desNeedlePose);

  const Eigen::Affine3d& currentNeedlePose = updateNeedlePose();
  if (currentNeedlePose.isApprox(desNeedlePose, 1e-2))
  {
    return true;
  }

  return m_pHandoffPlanner->localPlanObjectTransit(currentNeedlePose, ithTraj, jntTrajectoryBtwStates);
}
