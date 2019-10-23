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
 * Description: This is the derived valid state sampler inherited from ompl::base::StateValidityChecker
 */

#include <moveit/dual_arm_manipulation_planner_interface//hybrid_valid_state_sampler.h>

using namespace dual_arm_manipulation_planner_interface;

HybridValidStateSampler::HybridValidStateSampler(const std::string &robot_name,
                                                 const ompl::base::SpaceInformation* si)
  : robot_model_loader_(robot_name), ompl::base::ValidStateSampler(si)
{
  name_ = "hybrid valid state sampler";
  kmodel_ = robot_model_loader_.getModel();
}

bool HybridValidStateSampler::sample(ompl::base::State *state)
{
  HybridObjectStateSpace *hyStateSpace_ = si_->getStateSpace()->as<HybridObjectStateSpace>();
  ompl::base::DiscreteStateSampler arm_index_sampler(hyStateSpace_->getSubspace(1).get());
  ompl::base::DiscreteStateSampler grasp_index_sampler(hyStateSpace_->getSubspace(2).get());

  auto *hss = static_cast<HybridObjectStateSpace::StateType *>(state);

  arm_index_sampler.sampleUniform(&hss->armIndex());
  grasp_index_sampler.sampleUniform(&hss->graspIndex());

  robot_state::RobotStatePtr robot_sample_state;
  robot_sample_state.reset(new robot_state::RobotState(kmodel_));

  std::string planning_group = (hss->armIndex().value == 1) ? "psm_one" : "psm_two";
  const robot_state::JointModelGroup* selected_joint_model_group = robot_sample_state->getJointModelGroup(planning_group);
  robot_sample_state->setToRandomPositions(selected_joint_model_group);
  std::vector<double> joint_variables;
  robot_sample_state->copyJointGroupPositions(selected_joint_model_group, joint_variables);
  hyStateSpace_->setJointValues(joint_variables, hss);

  const Eigen::Affine3d tool_tip_pose = robot_sample_state->getGlobalLinkTransform(
    selected_joint_model_group->getOnlyOneEndEffectorTip());
  const Eigen::Affine3d grasp_pose = hyStateSpace_->possible_grasps_[hss->graspIndex().value].grasp_pose;
  const Eigen::Affine3d object_pose = tool_tip_pose * grasp_pose;
  hyStateSpace_->eigen3dToSE3(object_pose, hss);

  return si_->isValid(hss);
}
