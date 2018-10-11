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

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

using namespace dual_arm_manipulation_planner_interface;
using namespace ompl::base;

HybridObjectStateSpace::HybridObjectStateSpace(int armIndexLowerBound,
                                               int armIndexUpperBound,
                                               int graspIndexLowerBound,
                                               int graspIndexUpperBound,
                                               const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps)
  : ompl::base::CompoundStateSpace(), possible_grasps_(possible_grasps)
{
  setName("HybridObject" + getName());
  type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 10;

  addSubspace(std::make_shared<SE3StateSpace>(), 1.0);  // object pose space
  components_.back()->setName(components_.back()->getName() + ":ObjectPose");

  addSubspace(std::make_shared<DiscreteStateSpace>(armIndexLowerBound, armIndexUpperBound),
              1.0);  // arm index
  components_.back()->setName(components_.back()->getName() + ":ArmIndex");

  addSubspace(std::make_shared<DiscreteStateSpace>(graspIndexLowerBound, graspIndexUpperBound),
              1.0);  // grasp index
  components_.back()->setName(components_.back()->getName() + ":GraspIndex");

  lock();
}

void HybridObjectStateSpace::setArmIndexBounds(int lowerBound, int upperBound)
{
  components_[1]->as<DiscreteStateSpace>()->setBounds(lowerBound, upperBound);
}

void HybridObjectStateSpace::setGraspIndexBounds(int lowerBound, int upperBound)
{
  components_[2]->as<DiscreteStateSpace>()->setBounds(lowerBound, upperBound);
}

double HybridObjectStateSpace::distance(const State *state1, const State *state2) const
{
//  double se3_dist_trans, se3_dist_rot;

  double total_dist;
  double se3_dist;
  int num_handoff;

  const auto *hs1 = static_cast<const StateType *>(state1);
  const auto *hs2 = static_cast<const StateType *>(state2);

  se3_dist = components_[0]->distance(hs1->components[0], hs2->components[0]);

  const int s1_arm_index = hs1->armIndex().value;
  const int s2_arm_index = hs2->armIndex().value;

  const int s1_grasp_index = hs1->graspIndex().value;
  const int s2_grasp_index = hs2->graspIndex().value;

  const int s1_part_id = possible_grasps_[s1_grasp_index].part_id;
  const int s2_part_id = possible_grasps_[s2_grasp_index].part_id;

  if (s1_arm_index != s2_arm_index)
  {
    if (s1_part_id != s2_part_id)
    {
      num_handoff = 1;
    }
    else
    {
      num_handoff = 3;
    }
  }
  else  // s1_arm_index == s2_arm_index
  {
    num_handoff = 2;
  }

  total_dist = num_handoff + se3_dist;

  return total_dist;
}

State *HybridObjectStateSpace::allocState() const
{
  auto *state = new StateType();
  ompl::base::CompoundStateSpace::allocStateComponents(state);
  return static_cast<ompl::base::State *>(state);
}


void HybridObjectStateSpace::freeState(State *state) const
{
  ompl::base::CompoundStateSpace::freeState(state);
}

void HybridObjectStateSpace::copyState(State *destination, const State *source) const
{
  ompl::base::CompoundStateSpace::copyState(destination, source);
}

bool HybridObjectStateSpace::equalStates(const State *state1, const State *state2) const
{
  ompl::base::CompoundStateSpace::equalStates(state1, state2);
}

StateSamplerPtr HybridObjectStateSpace::allocDefaultStateSampler() const override
{
  return ompl::base::CompoundStateSpace::allocDefaultStateSampler();
}

StateSamplerPtr HybridObjectStateSpace::allocStateSampler() const override
{
  return ompl::base::CompoundStateSpace::allocDefaultStateSampler();
}

unsigned int HybridObjectStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
  const auto *hs1 = static_cast<const StateType *>(state1);
  const auto *hs2 = static_cast<const StateType *>(state2);

  const int s1_arm_index = hs1->armIndex().value;
  const int s2_arm_index = hs2->armIndex().value;

  const int s1_grasp_index = hs1->graspIndex().value;
  const int s2_grasp_index = hs2->graspIndex().value;

  const int s1_part_id = possible_grasps_[s1_grasp_index].part_id;
  const int s2_part_id = possible_grasps_[s2_grasp_index].part_id;

  if (s1_arm_index == s2_arm_index && s1_grasp_index == s2_grasp_index)
  {
    // need implementation
  }
}

bool HybridObjectStateSpace::discreteGeodesic(const State *from, const State *to, bool interpolate,
                                              std::vector<ompl::base::State *> *geodesic) const
{
  auto &&svc = si_->getStateValidityChecker();

  // We can't traverse the manifold if we don't start on it.
  if (!(interpolate || svc->isValid(from)))
    return false;

  auto afrom = from->as<StateType>();
  auto ato = to->as<StateType>();

  if (geodesic != nullptr)
  {
    geodesic->clear();
    geodesic->push_back(cloneState(from));
  }

  // No need to traverse the manifold if we are already there
  const double tolerance = delta_;
  const double distTo = distance(from, to);
  if (distTo <= tolerance)
    return true;



  // Create a scratch state to use for movement.
  auto scratch = cloneState(from)->as<StateType>();
  auto temp = allocState()->as<StateType>();



}

void HybridObjectStateSpace::interpolate(const State *from,
                                         const State *to,
                                         const double t,
                                         State *state) const
{
  auto *cstate = static_cast<StateType *>(state);

  const auto *hys_from = static_cast<const StateType *>(from);
  const auto *hys_to = static_cast<const StateType *>(to);

  components_[0]->interpolate(hys_from->components[0], hys_to->components[0], t,
                              cstate->components[0]);  // interpolation of se3 state sapce

  const int from_arm_index = hys_from->armIndex().value;
  const int to_arm_index = hys_to->armIndex().value;

  const int from_grasp_index = hys_from->graspIndex().value;
  const int to_grasp_index = hys_to->graspIndex().value;

  const int from_part_id = possible_grasps_[from_grasp_index].part_id;
  const int to_part_id = possible_grasps_[to_grasp_index].part_id;

  if (from_arm_index == to_arm_index)
  {

    if (from_grasp_index == to_grasp_index)
    {
      cstate->armIndex().value = from_arm_index;
      cstate->graspIndex().value = from_grasp_index;
    }
    else
    {
      cstate->armIndex().value = chooseSupportArm(from_arm_index, to_arm_index);
      cstate->graspIndex().value = chooseGraspPart(from_part_id, to_part_id);
    }
  }
  else
  {
    if (from_arm_index == 1)  // if from state is PSM1
    {
      cstate->armIndex().value = 2;  // cstate should be PSM2, may also be PSM1
      cstate->graspIndex().value = chooseGraspPart(from_part_id, to_part_id);
    }
    else if (from_arm_index == 2)  // if from state is PSM2
    {
      cstate->armIndex().value = 1;  // cstate should be PSM1, may also be PSM2
      cstate->graspIndex().value = chooseGraspPart(from_part_id, to_part_id);
    }
  }
}

int HybridObjectStateSpace::chooseSupportArm(int from_arm_index, int to_arm_index) const
{
  int cs_arm_id;

  if ((from_arm_index == to_arm_index) == 1)
  {
    cs_arm_id = 2;
  }
  else if (((from_arm_index == to_arm_index) == 2))
  {
    cs_arm_id = 1;
  }


  return cs_arm_id;
}

int HybridObjectStateSpace::chooseGraspPart(int from_part_id, int to_part_id) const
{
  int cs_grasp_id;

  for (int i = 0; i < possible_grasps_.size(); i++)
  {
    int cs_part_id = possible_grasps_[i]->part_id;
    if (cs_part_id != from_part_id && cs_part_id != to_part_id)
    {
      cs_grasp_id = i;
      break;
    }
  }

  return cs_grasp_id;
}