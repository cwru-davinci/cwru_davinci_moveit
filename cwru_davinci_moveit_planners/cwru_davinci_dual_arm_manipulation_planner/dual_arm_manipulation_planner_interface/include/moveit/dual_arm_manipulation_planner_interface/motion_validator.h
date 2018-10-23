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

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_STATE_VALIDITY_CHECKER_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_STATE_VALIDITY_CHECKER_H

#include <moveit/ompl_interface/detail/threadsafe_state_storage.h>
#include <moveit/ompl_interface/ompl_planning_context.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/Grasp.h>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>

#include <cwru_davinci_grasp/davinci_simple_grasp_generator.h>

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <cwru_davinci_dual_arm_manipulation_planner/state_validity_checker.h>

namespace dual_arm_manipulation_planner_interface
{

class MotionValidator : public ompl::base::MotionValidator
{
public:
  MotionValidator(const ros::NodeHandle &node_handle,
                  const ros::NodeHandle &node_priv,
                  const std::string &robot_name,
                  const std::string &object_name,
                  const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
                  const ompl::base::SpaceInformationPtr &si);

  virtual bool 	checkMotion (const State *s1, const State *s2) const = 0;

private:

  StateValidityChecker stateValidityChecker_;
};
}



#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_MOTION_VALIDATOR_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_MOTION_VALIDATOR_H

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_MOTION_VALIDATOR_H
