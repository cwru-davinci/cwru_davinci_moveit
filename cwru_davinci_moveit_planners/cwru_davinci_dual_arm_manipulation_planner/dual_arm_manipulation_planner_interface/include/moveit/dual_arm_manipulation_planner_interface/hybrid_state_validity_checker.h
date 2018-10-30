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
 * Description: This is the derived validity checker inherited from ompl::base::StateValidityChecker
 */

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_STATE_VALIDITY_CHECKER_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_STATE_VALIDITY_CHECKER_H

#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
//#include <moveit/ompl_interface/ompl_planning_context.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/Grasp.h>

#include <ompl/base/StateValidityChecker.h>

#include <cwru_davinci_grasp/davinci_simple_grasp_generator.h>
//#include <cwru_davinci_moveit_object_handling/davinci_object_message_generator.h>


namespace dual_arm_manipulation_planner_interface
{
class HybridStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  HybridStateValidityChecker(const ros::NodeHandle &node_handle,
                       const ros::NodeHandle &node_priv,
                       const std::string &robot_name,
                       const std::string &object_name,
                       const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
                       const ompl::base::SpaceInformationPtr &si);

  virtual ~HybridStateValidityChecker(){};

  virtual bool isValid(const ompl::base::State* state) const override
  {
    return isValid(state, verbose_);
  }

  bool isValid(const ompl::base::State* state, bool verbose) const;

  virtual double cost(const ompl::base::State* state) const override;

  virtual double clearance(const ompl::base::State* state) const override;

  void setVerbose(bool flag);

protected:
  void initializePlannerPlugin();

  bool isValidWithoutCache(const ompl::base::State* state, bool verbose) const;

  bool isValidWithCache(const ompl::base::State* state, bool verbose) const;

  void convertObjectToRobotState(robot_state::RobotState* robot_state, const ompl::base::State* state);

  /**
   * @brief Has certain arm group /@param group_name hold object /param object_name
   * @param group_name
   * @param object_name
   * @return
   */
  bool hasAttachedObject(const std::string& group_name, const std::string& object_name);

  ros::NodeHandle node_handle_;

  planning_interface::PlannerManagerPtr planner_instance_;

  planning_interface::PlanningSceneInterface planning_scene_interface_;

  planning_scene::PlanningScenePtr planning_scene_;

  planning_interface::PlanningContextPtr planning_context_;

  robot_model_loader::RobotModelLoader robot_model_loader_;

  robot_model::RobotModelPtr kmodel_;

  /// @brief Robot state containing the current position of all joints
  robot_state::RobotStatePtr complete_initial_robot_state_;

  TSStateStorage tss_;

//  DavinciObjectMessageGenerator objectMessageGenerator_;

  std::string group_name_;

  collision_detection::CollisionRequest collision_request_simple_;

  collision_detection::CollisionRequest collision_request_simple_verbose_;

  collision_detection::CollisionRequest collision_request_with_cost_;

  bool verbose_;

  std::vector<cwru_davinci_grasp::GraspInfo> possible_grasps_;

  std::string object_name_;
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_STATE_VALIDITY_CHECKER_H
