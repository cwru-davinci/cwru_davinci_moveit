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

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>

//#include <cwru_davinci_moveit_kinematics_plugin/davinci_moveit_kinematics_plugin.h>

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

  ~HybridMotionValidator() override = default;

  virtual bool checkMotion (const ompl::base::State *s1, const ompl::base::State *s2) const;

  virtual bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                           std::pair<ompl::base::State *, double> &lastValid) const
  {
    return false;
  }

  bool planHandoff(const robot_state::RobotState &start_state,
                   const robot_state::RobotState &goal_state,
                   const std::string &ss_active_group,
                   const std::string &gs_active_group) const;

  bool planNeedleGrasping(const robot_state::RobotState &start_state,
                          const robot_state::RobotState &handoff_state,
                          const std::string &gs_active_group) const;

  bool planNeedleReleasing(const robot_state::RobotState &handoff_state,
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

  bool noCollision(const robot_state::RobotState& rstate) const;

  bool planPathFromTwoStates(const robot_state::RobotState &start_state,
                             const robot_state::RobotState &goal_state,
                             const std::string &planning_group) const;

  void defaultSettings();

  void initializeIKPlugin();

  bool setFromIK(robot_state::RobotState &rstate,
                 const robot_state::JointModelGroup *arm_joint_group,
                 const std::string &planning_group,
                 const std::string &tip_frame,
                 const Eigen::Affine3d& tip_pose_wrt_world) const;

//  void publishRobotState(const robot_state::RobotState& rstate) const;

  HybridObjectStateSpace *hyStateSpace_;

  HybridStateValidityChecker stateValidityChecker_;

  robot_model::RobotModelPtr kmodel_;

  robot_model_loader::RobotModelLoader robot_model_loader_;

  planning_interface::PlannerManagerPtr planner_instance_;

  planning_scene::PlanningScenePtr planning_scene_;

//  planning_scene_monitor::PlanningSceneMonitorPtr pMonitor_;

  ros::NodeHandle node_handle_;

  ros::NodeHandle node_priv_;

  std::string robot_name_;

  std::string object_name_;

  boost::shared_ptr<kinematics::KinematicsBase> psm_one_kinematics_solver_;
  boost::shared_ptr<kinematics::KinematicsBase> psm_two_kinematics_solver_;  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> >
    kinematics_loader_;

  ros::Publisher robot_state_publisher_;
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_MOTION_VALIDATOR_H
