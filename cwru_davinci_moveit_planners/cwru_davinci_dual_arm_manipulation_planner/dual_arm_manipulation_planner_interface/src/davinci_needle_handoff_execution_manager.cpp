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
}

DavinciNeedleHandoffExecutionManager::~DavinciNeedleHandoffExecutionManager
(
)
{
  m_pHandoffPlanner->m_pHyStateSpace->freeState(m_pHyStartState);
  m_pHandoffPlanner->m_pHyStateSpace->freeState(m_pHyGoalState);
}

bool DavinciNeedleHandoffExecutionManager::executeNeedleHandoffTrajy
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

  for (std::size_t i = 0; i < m_HandoffJntTraj.size(); ++i)
  {
    if (m_HandoffJntTraj[i].size() == 1)  // object Transit
    {
      // move
      const MoveGroupJointTrajectorySegment& jntTrajSeg = m_HandoffJntTraj[i][0].second;
      m_pSupportArmGroup.reset(new MoveGroupInterface(jntTrajSeg.begin()->first));
      const JointTrajectory& jntTra = jntTrajSeg.begin()->second;
      for(std::size_t j; j < jntTra.size(); ++j)
      {
        m_pSupportArmGroup->setJointValueTarget(jntTra[j]);
        errorCodes = m_pSupportArmGroup->move();
        if (errorCodes.val != errorCodes.SUCCESS)
        {
          ROS_INFO("DavinciNeedleHandoffExecutionManager: "
                   "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
          return false;
        }
      }
    }
    else if (m_HandoffJntTraj[i].size() == 4)  // object Transfer
    {
      // move in fashion: home to pregrasp, approach-grasp, ungrasp-retreat, back to home
      const MoveGroupJointTrajectorySegment& safePlaceToPreGraspJntTrajSeg = m_HandoffJntTraj[i][0].second;
      m_pSupportArmGroup.reset(new MoveGroupInterface(safePlaceToPreGraspJntTrajSeg.begin()->first));
      {
        const JointTrajectory& jntTra = safePlaceToPreGraspJntTrajSeg.begin()->second;
        for(std::size_t j; j < jntTra.size(); ++j)
        {
          m_pSupportArmGroup->setJointValueTarget(jntTra[j]);
          m_pSupportArmGroup->move();
          errorCodes = m_pSupportArmGroup->move();
          if (errorCodes.val != errorCodes.SUCCESS)
          {
            ROS_INFO("DavinciNeedleHandoffExecutionManager: "
                     "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
            return false;
          }
        }
      }

      const MoveGroupJointTrajectorySegment& preGraspToGraspedJntTrajSeg = m_HandoffJntTraj[i][1].second;
      m_pSupportArmGroup.reset(new MoveGroupInterface(preGraspToGraspedJntTrajSeg.begin()->first));
      m_pSupportArmEefGroup.reset(new MoveGroupInterface((++preGraspToGraspedJntTrajSeg.begin())->first));
      {
        const JointTrajectory& armJntTra = preGraspToGraspedJntTrajSeg.begin()->second;
        for(std::size_t j; j < armJntTra.size(); ++j)
        {
          m_pSupportArmGroup->setJointValueTarget(armJntTra[j]);
          m_pSupportArmGroup->move();
          errorCodes = m_pSupportArmGroup->move();
          if (errorCodes.val != errorCodes.SUCCESS)
          {
            ROS_INFO("DavinciNeedleHandoffExecutionManager: "
                     "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
            return false;
          }
        }

        const JointTrajectory& gripperJntTra = (++preGraspToGraspedJntTrajSeg.begin())->second;
        for(std::size_t j; j < gripperJntTra.size(); ++j)
        {
          m_pSupportArmEefGroup->setJointValueTarget(gripperJntTra[j]);
          m_pSupportArmEefGroup->move();
          errorCodes = m_pSupportArmGroup->move();
          if (errorCodes.val != errorCodes.SUCCESS)
          {
            ROS_INFO("DavinciNeedleHandoffExecutionManager: "
                     "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
            return false;
          }
        }
      }

      const MoveGroupJointTrajectorySegment& graspToUngraspedJntSeg = m_HandoffJntTraj[i][2].second;
      m_pSupportArmGroup.reset(new MoveGroupInterface(graspToUngraspedJntSeg.begin()->first));
      {
        const JointTrajectory& jntTra = graspToUngraspedJntSeg.begin()->second;
        for(std::size_t j; j < jntTra.size(); ++j)
        {
          m_pSupportArmGroup->setJointValueTarget(jntTra[j]);
          m_pSupportArmGroup->move();
          errorCodes = m_pSupportArmGroup->move();
          if (errorCodes.val != errorCodes.SUCCESS)
          {
            ROS_INFO("DavinciNeedleHandoffExecutionManager: "
                     "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
            return false;
          }
        }
      }

      const MoveGroupJointTrajectorySegment& ungraspedToSafePlaceJntTrajSeg = m_HandoffJntTraj[i][3].second;
      m_pSupportArmGroup.reset(new MoveGroupInterface(ungraspedToSafePlaceJntTrajSeg.begin()->first));
      m_pSupportArmEefGroup.reset(new MoveGroupInterface((++ungraspedToSafePlaceJntTrajSeg.begin())->first));
      {
        const JointTrajectory& armJntTra = ungraspedToSafePlaceJntTrajSeg.begin()->second;
        for(std::size_t j; j < armJntTra.size(); ++j)
        {
          m_pSupportArmGroup->setJointValueTarget(armJntTra[j]);
          m_pSupportArmGroup->move();
          errorCodes = m_pSupportArmGroup->move();
          if (errorCodes.val != errorCodes.SUCCESS)
          {
            ROS_INFO("DavinciNeedleHandoffExecutionManager: "
                     "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
            return false;
          }
        }

        const JointTrajectory& gripperJntTra = (++ungraspedToSafePlaceJntTrajSeg.begin())->second;
        for(std::size_t j; j < gripperJntTra.size(); ++j)
        {
          m_pSupportArmEefGroup->setJointValueTarget(gripperJntTra[j]);
          m_pSupportArmEefGroup->move();
          errorCodes = m_pSupportArmGroup->move();
          if (errorCodes.val != errorCodes.SUCCESS)
          {
            ROS_INFO("DavinciNeedleHandoffExecutionManager: "
                     "Failed to execute handoff trajectories, moveit error code is %d", errorCodes.val);
            return false;
          }
        }
      }
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
const ompl::base::SE3StateSpace::StateType* objGoalPose,
const int goalSupportArmIdx,
const int goalGraspIdx
)
{
  if(!m_pHandoffPlanner->m_pHyStateSpace)
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: invalid hybrid state space pointer");
    return false;
  }

  m_pHyStartState = m_pHandoffPlanner->m_pHyStateSpace->allocState()->as<HybridObjectStateSpace::StateType>();
  m_pHyGoalState = m_pHandoffPlanner->m_pHyStateSpace->allocState()->as<HybridObjectStateSpace::StateType>();

  m_pHyStartState->se3State().setXYZ(objStartPose->getX(),
                                     objStartPose->getY(),
                                     objStartPose->getZ());
  m_pHyStartState->se3State().rotation().x = objStartPose->rotation().x;
  m_pHyStartState->se3State().rotation().y = objStartPose->rotation().y;
  m_pHyStartState->se3State().rotation().z = objStartPose->rotation().z;
  m_pHyStartState->se3State().rotation().w = objStartPose->rotation().w;

  m_pHyStartState->armIndex().value = startSupportArmIdx;
  m_pHyStartState->graspIndex().value = startGraspIdx;

  m_pHyGoalState->se3State().setXYZ(objGoalPose->getX(),
                                    objGoalPose->getY(),
                                    objGoalPose->getZ());
  m_pHyGoalState->se3State().rotation().x = objGoalPose->rotation().x;
  m_pHyGoalState->se3State().rotation().y = objGoalPose->rotation().y;
  m_pHyGoalState->se3State().rotation().z = objGoalPose->rotation().z;
  m_pHyGoalState->se3State().rotation().w = objGoalPose->rotation().w;

  m_pHyGoalState->armIndex().value = goalSupportArmIdx;
  m_pHyGoalState->graspIndex().value = goalGraspIdx;

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
      m_ArmIndexBounds[i] = static_cast<double>(xmlArmIndexBounds[i]);
    }
  }
  else
  {
    ROS_ERROR_STREAM("SE3 bounds type is not type array?");
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

  if(m_GraspInfo.empty())
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: input grasp info list is empty");
    return false;
  }

  if(!m_pHandoffPlanner)
  {
    ROS_ERROR("DavinciNeedleHandoffExecutionManager: invalid handoff planner pointer");
    return false;
  }

  if (!m_pHandoffPlanner->m_pHyStateSpace)
  {
    m_pHandoffPlanner->setupStateSpace(m_GraspInfo);
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

  m_pHandoffPlanner->setupProblemDefinition(m_pHyStartState, m_pHyGoalState);
  m_pHandoffPlanner->setupPlanner(maxDistance);
  return true;
}
