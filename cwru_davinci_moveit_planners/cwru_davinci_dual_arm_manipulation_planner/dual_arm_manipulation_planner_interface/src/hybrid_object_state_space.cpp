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
 * Description: This hybrid object state space is compound state space of SE3 state space
 * in addition of two discrete state space (Arm index and Grasp index)
 */

#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"
#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

ompl::base::HybridObjectStateSpace::HybridObjectStateSpace(int armIndexLowerBound,
                                                           int armIndexUpperBound,
                                                           int graspIndexLowerBound,
                                                           int graspIndexUpperBound) : CompoundStateSpace()
{
  setName("HybridObject" + getName());
  type_ = STATE_SPACE_TYPE_COUNT + 1;

  addSubspace(std::make_shared<SE3StateSpace>(), 1.0);  // object pose space
  components_.back()->setName(components_.back()->getName() + ":ObjectPose");

  addSubspace(std::make_shared<DiscreteStateSpace>(armIndexLowerBound, armIndexUpperBound), 1.0);  // arm index
  components_.back()->setName(components_.back()->getName() + ":ArmIndex");

  addSubspace(std::make_shared<DiscreteStateSpace>(graspIndexLowerBound, graspIndexUpperBound), 1.0);  // grasp index
  components_.back()->setName(components_.back()->getName() + ":GraspIndex");

  state_space_.reset(new CompoundStateSpace());

  lock();
}

void ompl::base::HybridObjectStateSpace::setArmIndexBounds(int lowerBound, int upperBound)
{
  components_[1]->as<DiscreteStateSpace>()->setBounds(lowerBound, upperBound);
}

void ompl::base::HybridObjectStateSpace::setGraspIndexBounds(int lowerBound, int upperBound)
{
  components_[2]->as<DiscreteStateSpace>()->setBounds(lowerBound, upperBound);
}

double ompl::base::HybridObjectStateSpace::distance(const State *state1, const State *state2) const
{
  double se3_dist_trans, se3_dist_rot;
  int num_handoff;

  const auto *ps1 = static_cast<const HybridObjectStateSpace::StateType *>(state1);
  const auto *ps2 = static_cast<const HybridObjectStateSpace::StateType *>(state2);

  se3_dist_trans = distance(ps1->se3State().components[0], ps2->se3State().components[0]);
  se3_dist_rot = distance(ps1->se3State().components[1], ps2->se3State().components[1]);

//  num_handoff =
}

ompl::base::State* ompl::base::HybridObjectStateSpace::allocState() const
{
  auto *state = new StateType();
  allocStateComponents(state);
  return state;
}

const ompl::base::RealVectorBounds ompl::base::HybridObjectStateSpace::getBounds() const
{
  return as<SE3StateSpace>(0)->getBounds();
};