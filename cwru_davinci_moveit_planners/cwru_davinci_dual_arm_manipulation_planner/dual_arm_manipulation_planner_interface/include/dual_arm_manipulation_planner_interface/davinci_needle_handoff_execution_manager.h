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

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DAVINCI_NEEDLE_HANDOFF_EXECUTION_MANAGER_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DAVINCI_NEEDLE_HANDOFF_EXECUTION_MANAGER_H

//moveit
//#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>

//#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit_msgs/DisplayTrajectory.h>
//
//#include <moveit_msgs/AttachedCollisionObject.h>
//#include <moveit_msgs/CollisionObject.h>

//#include <moveit_visual_tools/moveit_visual_tools.h>

//ompl
#include <ompl/geometric/PathGeometric.h>

// cwru_davinci_grasp
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>

namespace dual_arm_manipulation_planner_interface
{
enum class TrajDiff
{
  SameArm,
  DiffArm,
};

struct HandoffSlnPath
{
  std::string support_arm;
  std::string rest_arm;
  int grasp_id;
  double* joint_values;
  geometry_msgs::PoseStamped needle_pose;

  HandoffSlnPath(int dim)
  {
    joint_values = new double[dim];
  }

  ~HandoffSlnPath()
  {
    delete[] joint_values;
  }
};

class DavinciNeedleHandoffExecution
{
public:
  DavinciNeedleHandoffExecution(const ompl::geometric::PathGeometric &sln_path,
                                int joint_space_dim,
                                const std::vector<cwru_davinci_grasp::GraspInfo> &grasp_poses,
                                const cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr &needleGrasper);

  bool executeNeedleHandoffTrajy();

private:
//  void defaultSettings();

  bool readData();

  bool executeNeedleTransfer(const HandoffSlnPath& ss_joint_traj, const HandoffSlnPath& gs_joint_traj);

  bool executeNeedleTransit(const HandoffSlnPath& ss_joint_traj, const HandoffSlnPath& gs_joint_traj);

  bool executeNeedleGrasping(const HandoffSlnPath& joint_traj);

  bool executeNeedleUngrasping(const HandoffSlnPath& joint_traj);

  TrajDiff checkTrajDiff(const std::string& support_arm_1, const std::string& support_arm_2);

  std::vector<HandoffSlnPath> handoff_traj_sequence_;

  std::vector<ompl::base::State*> handoff_state_;

  ompl::geometric::PathGeometric sln_path_;

  int joint_space_dim_;

  std::vector<cwru_davinci_grasp::GraspInfo> grasp_poses_;

  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr needleGrasper_;
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DAVINCI_NEEDLE_HANDOFF_EXECUTION_MANAGER_H