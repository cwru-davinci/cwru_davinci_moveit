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

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_STATE_VALIDITY_CHECKER_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_STATE_VALIDITY_CHECKER_H

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <moveit/dual_arm_manipulation_planner_interface/threadsafe_state_storage.h>
// moveit
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// ompl
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>

#include <chrono>  // for high_resolution_clock

namespace dual_arm_manipulation_planner_interface
{

class HybridStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
  HybridStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                             const moveit::core::RobotModelConstPtr &pRobotModel,
                             const std::string &object_name);

  virtual ~HybridStateValidityChecker() {}

  virtual bool isValid(const ompl::base::State* state) const override;

  virtual double cost(const ompl::base::State* state) const;

  virtual double clearance(const ompl::base::State* state) const override;

  bool convertObjectToRobotState(const robot_state::RobotStatePtr &pRSstate,
                                 const HybridObjectStateSpace::StateType *pHyState,
                                 const std::string &planning_group) const;

  moveit::core::AttachedBody *createAttachedBody(const std::string &active_group,
                                                 const std::string &object_name,
                                                 const int grasp_pose_id) const;

protected:
  void defaultSettings();

  void loadNeedleModel();

  void publishRobotState(const robot_state::RobotState& rstate) const;

  void setMimicJointPositions(const robot_state::RobotStatePtr &rstate,
                              const std::string &planning_group) const;
protected:
  HybridObjectStateSpace*                   hyStateSpace_;

  planning_scene::PlanningScenePtr          planning_scene_;

  robot_model::RobotModelConstPtr           kmodel_;

  collision_detection::CollisionRequest     collision_request_simple_;

  collision_detection::CollisionRequest     collision_request_with_cost_;

  collision_detection::CollisionRequest     collision_request_with_distance_;

  std::string                               object_name_;

  std::vector<shapes::ShapeConstPtr>        needleShapes_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
};

}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_STATE_VALIDITY_CHECKER_H
