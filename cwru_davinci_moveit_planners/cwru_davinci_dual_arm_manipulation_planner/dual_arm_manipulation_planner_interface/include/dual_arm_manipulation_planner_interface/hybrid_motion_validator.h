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

#include <dual_arm_manipulation_planner_interface/hybrid_state_validity_checker.h>

// ompl
#include <ompl/base/MotionValidator.h>

// eigen
//#include <Eigen/Core>
//#include <fstream>

namespace dual_arm_manipulation_planner_interface
{

class HybridMotionValidator : public ompl::base::MotionValidator, public HybridStateValidityChecker
{
public:
  HybridMotionValidator
  (
  const ompl::base::SpaceInformationPtr& si,
  const robot_model::RobotModelConstPtr& pRobotModel,
  const std::string& objectName
  );

  virtual ~HybridMotionValidator()
  {}

  virtual bool checkMotion
  (
  const ompl::base::State* s1,
  const ompl::base::State* s2
  ) const;

  virtual bool checkMotion
  (
  const ompl::base::State* s1,
  const ompl::base::State* s2,
  std::pair<ompl::base::State*, double>& lastValid
  ) const
  {
    return false;
  }

  bool planHandoff
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& goal_state,
  const std::string& ss_active_group,
  const std::string& gs_active_group
  ) const;

  bool planNeedleGrasping
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& handoff_state,
  const std::string& gs_active_group
  ) const;

  bool planNeedleReleasing
  (
  const robot_state::RobotState& handoff_state,
  const robot_state::RobotState& goal_state,
  const std::string& ss_active_group
  ) const;

protected:
  bool planSafeStateToPreGraspState
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& pre_grasp_state,
  const std::string& planning_group
  ) const;

  bool planPreGraspStateToGraspedState
  (
  robot_state::RobotStatePtr& pre_grasp_state,
  const robot_state::RobotState& handoff_state,
  const std::string& planning_group
  ) const;

  bool planGraspStateToUngraspedState
  (
  const robot_state::RobotState& handoff_state,
  robot_state::RobotStatePtr& ungrasped_state,
  const std::string& planning_group
  ) const;

  bool planUngraspedStateToSafeState
  (
  const robot_state::RobotState& ungrasped_state,
  const robot_state::RobotState& goal_state,
  const std::string& planning_group
  ) const;

  bool planObjectTransit
  (
  const robot_state::RobotState& start_state,
  const robot_state::RobotState& goal_state,
  const std::string& planning_group
  ) const;

protected:
  ros::NodeHandle                           node_priv_;
};

}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_MOTION_VALIDATOR_H
