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
const ros::NodeHandle &nodeHandlePrivate
)
: m_NodeHandlePrivate(nodeHandlePrivate)
{

}

bool DavinciNeedleHandoffExecutionManager::executeNeedleHandoffTrajy
(
)
{
  if (m_PlanningStatus == ob::PlannerStatus::EXACT_SOLUTION && !m_HandoffJntTraj.empty())
  {
    for (std::size_t i = 0; i < m_HandoffJntTraj.size(); ++i)
    {
      
    }
  }
  else
  {
    return false;
  }
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