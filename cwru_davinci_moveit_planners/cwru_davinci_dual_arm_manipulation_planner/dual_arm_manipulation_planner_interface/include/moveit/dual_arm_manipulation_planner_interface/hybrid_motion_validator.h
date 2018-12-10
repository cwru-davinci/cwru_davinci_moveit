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
 * Description: This is the derived motion validator inherited from ompl::base::MotionValidator
 */

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_MOTION_VALIDATOR_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_MOTION_VALIDATOR_H

#include <moveit/dual_arm_manipulation_planner_interface/hybrid_state_validity_checker.h>
#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/Grasp.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>

#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>

namespace dual_arm_manipulation_planner_interface
{

class HybridMotionValidator : public ompl::base::MotionValidator
{
public:
  HybridMotionValidator(const ros::NodeHandle &node_handle,
                        const ros::NodeHandle &node_priv,
                        const std::string &robot_name,
                        const std::string &object_name,
                        const ompl::base::SpaceInformationPtr &si);

  virtual bool checkMotion (const ompl::base::State *s1, const ompl::base::State *s2) const = 0;

  bool planHandoff(const robot_state::RobotState &start_state,
                   const robot_state::RobotState &goal_state,
                   const std::string &ss_active_group,
                   const std::string &gs_active_group) const;

  bool planNeedleGrasping(const robot_state::RobotState &start_state,
                          const robot_state::RobotState &goal_state,
                          const std::string &gs_active_group) const;

  bool planNeedleReleasing(const robot_state::RobotState &start_state,
                           const robot_state::RobotState &goal_state,
                           const std::string &ss_active_group) const;

private:
  void initializePlannerPlugin();

  bool planSafeStateToPreGraspState(const robot_state::RobotState &start_state,
                                    const robot_state::RobotState &pre_grasp_state,
                                    const std::string &planning_group) const;

  bool planPreGraspStateToGraspedState(robot_state::RobotState &pre_grasp_state,
                                       const robot_state::RobotState &handoff_state,
                                       const std::string &planning_group) const;

  bool planGraspStateToUngraspedState(const robot_state::RobotState &handoff_state,
                                      robot_state::RobotState &ungrasped_state,
                                      const std::string &planning_group) const;

  bool planUngraspedStateToSafeState(const robot_state::RobotState &ungrasped_state,
                                     const robot_state::RobotState &goal_state,
                                     const std::string &planning_group) const;


  bool planPathFromTwoStates(const robot_state::RobotState &start_state,
                             const robot_state::RobotState &goal_state,
                             const std::string &planning_group) const;

  HybridStateValidityChecker stateValidityChecker_;

  robot_model::RobotModelPtr kmodel_;

  robot_model_loader::RobotModelLoader robot_model_loader_;

  planning_interface::PlannerManagerPtr planner_instance_;

  planning_scene::PlanningScenePtr planning_scene_;

  planning_scene_monitor::PlanningSceneMonitorPtr pMonitor_;

//  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr needleGrasperPtr_;
//
//  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr needleReleasePtr_;

  ros::NodeHandle node_handle_;

  ros::NodeHandle node_priv_;

  std::string robot_name_;

  std::string object_name_;
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_MOTION_VALIDATOR_H
