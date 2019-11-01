/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Case Western Reserve University
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
const ros::NodeHandle& nodeHandlePrivate
)
:m_NodeHandle(nodeHandle), m_NodeHandlePrivate(nodeHandlePrivate)
{
//  m_pNeedleGrasper(new )
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
        printf("DavinciNeedleHandoffExecutionManager: Failed to generate handoff trajectories");
        return false;
      }
    }
    else
    {
      printf("DavinciNeedleHandoffExecutionManager: Needle handoff planning failed");
      m_HandoffJntTraj.resize(0);
      return false;
    }
  }

}