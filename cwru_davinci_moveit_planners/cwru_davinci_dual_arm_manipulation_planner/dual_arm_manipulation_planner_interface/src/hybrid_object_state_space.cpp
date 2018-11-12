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

void HybridObjectStateSpace::setSE3Bounds(const ompl::base::RealVectorBounds &bounds)
{
  components_[0]->as<SE3StateSpace>()->setBounds(bounds);
}

void HybridObjectStateSpace::setArmIndexBounds(int lowerBound, int upperBound)
{
  components_[1]->as<DiscreteStateSpace>()->setBounds(lowerBound, upperBound);
}

void HybridObjectStateSpace::setGraspIndexBounds(int lowerBound, int upperBound)
{
  components_[2]->as<DiscreteStateSpace>()->setBounds(lowerBound, upperBound);
}

bool HybridObjectStateSpace::isHybrid() const
{
  return true;
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

StateSamplerPtr HybridObjectStateSpace::allocDefaultStateSampler() const
{
  return ompl::base::CompoundStateSpace::allocDefaultStateSampler();
}

StateSamplerPtr HybridObjectStateSpace::allocStateSampler() const
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

  unsigned int se3_count = ompl::base::CompoundStateSpace::validSegmentCount(hs1->components[0], hs2->components[0]);
  unsigned int handoff_count = 0;

  if (s1_arm_index == s2_arm_index && s1_grasp_index == s2_grasp_index)
  {
    return se3_count;
  }
  else
  {
    if (s1_arm_index != s2_arm_index)
    {
      if (s1_part_id != s2_part_id)
      {
        handoff_count = 1;
        return handoff_count + se3_count;
      }
      else
      {
        handoff_count = 3;
        return handoff_count + se3_count;
      }
    }
    else  // s1_arm_index == s2_arm_index
    {
      handoff_count = 2;
      return handoff_count + se3_count;
    }
  }
}

//bool HybridObjectStateSpace::discreteGeodesic(const State *from, const State *to, bool interpolate,
//                                              std::vector<ompl::base::State *> *geodesic) const
//{
//  auto &&svc = si_->getStateValidityChecker();
//
//  // We can't traverse the manifold if we don't start on it.
//  if (!(interpolate || svc->isValid(from)))
//    return false;
//
//  auto afrom = from->as<StateType>();
//  auto ato = to->as<StateType>();
//
//  if (geodesic != nullptr)
//  {
//    geodesic->clear();
//    geodesic->push_back(cloneState(from));
//  }
//
//  // No need to traverse the manifold if we are already there
//  const double tolerance = delta_;
//  const double distTo = distance(from, to);
//  if (distTo <= tolerance)
//    return true;
//
//
//
//  // Create a scratch state to use for movement.
//  auto scratch = cloneState(from)->as<StateType>();
//  auto temp = allocState()->as<StateType>();
//
//
//
//}


//bool HybridObjectStateSpace::computeStateFK(ompl::base::State *state) const
//{
//
//}
//
//bool HybridObjectStateSpace::computeStateIK(ompl::base::State *state) const
//{
//  if (state->as<StateType>()->jointsComputed())
//    return true;
//  for (std::size_t i = 0 ; i < poses_.size() ; ++i)
//    if (!poses_[i].computeStateIK(state->as<StateType>(), i))
//    {
//      state->as<StateType>()->markInvalid();
//      return false;
//    }
//  state->as<StateType>()->setJointsComputed(true);
//  return true;
//}
//
//bool HybridObjectStateSpace::computeStateK(ompl::base::State *state) const
//{
//  if (state->as<StateType>()->jointsComputed() && !state->as<StateType>()->poseComputed())
//    return computeStateFK(state);
//  if (!state->as<StateType>()->jointsComputed() && state->as<StateType>()->poseComputed())
//    return computeStateIK(state);
//  if (state->as<StateType>()->jointsComputed() && state->as<StateType>()->poseComputed())
//    return true;
//  state->as<StateType>()->markInvalid();
//  return false;
//}
//
//bool HybridObjectStateSpace::computeStateIK(StateType *state) const
//{
//  // construct the pose
//  geometry_msgs::Pose pose;
//  const ompl::base::SE3StateSpace::StateType *se3_state = full_state->poses[idx];
//  pose.position.x = se3_state->getX();
//  pose.position.y = se3_state->getY();
//  pose.position.z = se3_state->getZ();
//  const ompl::base::SO3StateSpace::StateType &so3_state = se3_state->rotation();
//  pose.orientation.x = so3_state.x;
//  pose.orientation.y = so3_state.y;
//  pose.orientation.z = so3_state.z;
//  pose.orientation.w = so3_state.w;
//
//  // run IK
//  std::vector<double> solution(kinematics_solver_->getJointNames().size(), 0.0);
//  std::vector<double> seed(kinematics_solver_->getJointNames().size(), 0.0);
//  moveit_msgs::MoveItErrorCodes err_code;
//  if (!kinematics_solver_->searchPositionIK(pose, seed, timeout, solution,
//                                                                           error_code))
//  {
//    if (err_code.val != moveit_msgs::MoveItErrorCodes::TIMED_OUT ||
//        !kinematics_solver_->searchPositionIK(pose, seed_values, kinematics_solver_->getDefaultTimeout() * 2.0, solution, err_code))
//      return false;
//  }
//
//  for (std::size_t i = 0 ; i < bijection_.size() ; ++i)
//    full_state->values[bijection_[i]] = solution[i];
//
//  return true;
//}

void HybridObjectStateSpace::interpolate(const ompl::base::State *from,
                                         const ompl::base::State *to,
                                         const double t,
                                         ompl::base::State *state) const
{
  auto *cstate = static_cast<StateType *>(state);

  const auto *hys_from = static_cast<const StateType *>(from);
  const auto *hys_to = static_cast<const StateType *>(to);

//  cstate->components[0] = hys_from->components[0];
//
//  const int from_arm_index = hys_from->armIndex().value;
//  const int to_arm_index = hys_to->armIndex().value;
//
//  const int from_grasp_index = hys_from->graspIndex().value;
//  const int to_grasp_index = hys_to->graspIndex().value;
//
//  const int from_part_id = possible_grasps_[hys_from->graspIndex().value].part_id;
//  const int to_part_id = possible_grasps_[hys_to->graspIndex().value].part_id;

//  StateDiff stateDiff = checkStateDiff(hys_from, hys_to);

  switch (checkStateDiff(hys_from, hys_to))
  {
    case StateDiff::AllSame:
      ompl::base::CompoundStateSpace::copyState(cstate, hys_to);
      break;
    case StateDiff::ArmDiffGraspAndPoseSame:
      ompl::base::CompoundStateSpace::copyState(cstate->components[0], hys_from->components[0]);
      chooseGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::GraspDiffArmAndPoseSame:
      ompl::base::CompoundStateSpace::copyState(cstate->components[0], hys_from->components[0]);
      chooseGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::PoseDiffArmAndGraspSame:
      ompl::base::CompoundStateSpace::interpolate(hys_from->components[0], hys_to->components[0], t, cstate->components[0]);
      components_[1]->copyState(hys_from->components[1], cstate->components[1]);
      components_[2]->copyState(hys_from->components[2], cstate->components[2]);
      break;
    case StateDiff::ArmAndGraspDiffPoseSame:
      ompl::base::CompoundStateSpace::copyState(cstate->components[0], hys_from->components[0]);
      chooseGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::ArmAndPoseDiffGraspSame:
      ompl::base::CompoundStateSpace::copyState(cstate->components[0], hys_from->components[0]);
      chooseGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::GraspAndPoseDiffArmSame:
      ompl::base::CompoundStateSpace::copyState(cstate->components[0], hys_from->components[0]);
      chooseGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::AllDiff:
      ompl::base::CompoundStateSpace::copyState(cstate->components[0], hys_from->components[0]);
      chooseGrasp(hys_from, hys_to, cstate);
      break;
  }
}


void HybridObjectStateSpace::se3ToEign3d(const StateType *state, Eigen::Affine3d& affine3d) const
{
  affine3d.translation() << state->se3State().getX(), state->se3State().getY(), state->se3State().getZ();
  affine3d.rotate(Eigen::Quaterniond(state->se3State().rotation().w,
                                     state->se3State().rotation().x,
                                     state->se3State().rotation().y,
                                     state->se3State().rotation().z));
}

StateDiff HybridObjectStateSpace::checkStateDiff(const StateType *state1, const StateType *state2) const
{
  bool same_pose = ompl::base::CompoundStateSpace::equalStates(state1->components[0], state2->components[0]);
  bool same_arm = components_[1]->equalStates(state1->components[1], state2->components[1]);
  bool same_grasp = components_[2]->equalStates(state1->components[2], state2->components[2]);

  if(same_arm)
  {
    if(same_grasp)
    {
      if(!same_pose)
        return StateDiff::PoseDiffArmAndGraspSame;
      else
        return StateDiff::AllSame;
    }
    else
    {
      if(same_pose)
        return StateDiff::GraspDiffArmAndPoseSame;
      else
        return StateDiff::GraspAndPoseDiffArmSame;
    }
  }
  else
  {
    if(same_grasp)
    {
      if(same_pose)
        return StateDiff::ArmDiffGraspAndPoseSame;
      else
        return StateDiff::ArmAndPoseDiffGraspSame;
    }
    else
    {
      if(same_pose)
        return StateDiff::ArmAndGraspDiffPoseSame;
      else
        return StateDiff::AllDiff;
    }
  }
}


void HybridObjectStateSpace::chooseGrasp(const StateType *from,
                                         const StateType *to,
                                         StateType* cstate) const
{
  const int from_arm_index = from->armIndex().value;
  const int to_arm_index = to->armIndex().value;

  const int from_grasp_index = from->graspIndex().value;
  const int to_grasp_index = to->graspIndex().value;

  const int from_part_id = possible_grasps_[from_grasp_index].part_id;
  const int to_part_id = possible_grasps_[to_grasp_index].part_id;

  switch(handOffsNum(from_arm_index, to_arm_index, from_part_id, to_part_id))
  {
    case 1:
      cstate->armIndex().value = to_arm_index;
      cstate->graspIndex().value = to_grasp_index;
      break;
    case 2:
      cstate->armIndex().value = chooseSupportArm(from_arm_index, to_arm_index);
      cstate->graspIndex().value = chooseGraspPart(from_part_id, to_part_id);
      break;
    case 3:
      cstate->armIndex().value = to_arm_index;
      cstate->graspIndex().value = chooseGraspPart(from_part_id, to_part_id);
      break;
  }
}


int HybridObjectStateSpace::handOffsNum(const int from_arm_index,
                                        const int to_arm_index,
                                        const int from_part_id,
                                        const int to_part_id) const
{
  int num_handoff = 0;

  if (from_arm_index != to_arm_index)
  {
    if (from_part_id != to_part_id)  // case 1 time of handoff
    {
      num_handoff = 1;
      return num_handoff;
    }
    else  // case 3 time of handoffs
    {
      num_handoff = 3;
      return num_handoff;
    }
  }
  else  // s1_arm_index == s2_arm_index
  {
    num_handoff = 2;
    return num_handoff;
  }
}

int HybridObjectStateSpace::chooseSupportArm(const int from_arm_index, const int to_arm_index) const
{
  if ((from_arm_index == to_arm_index) == 1)
  {
    return 2;
  }
  else if (((from_arm_index == to_arm_index) == 2))
  {
    return 1;
  }
}

int HybridObjectStateSpace::chooseGraspPart(int from_part_id, int to_part_id) const
{
  int cs_grasp_id;

  ompl::base::DiscreteStateSampler grasp_index_sampler(components_[2].get());
  ompl::base::DiscreteStateSpace::StateType *temp_state = components_[2]->allocState()->as<ompl::base::DiscreteStateSpace::StateType>();

  grasp_index_sampler.sampleUniform(temp_state);
  int grasp_part = possible_grasps_[temp_state->value].part_id;

  while((grasp_part == from_part_id) || (grasp_part == to_part_id))
  {
    grasp_index_sampler.sampleUniform(temp_state);
    grasp_part = possible_grasps_[temp_state->value].part_id;
  }

  cs_grasp_id = temp_state->value;
  components_[2]->freeState(temp_state);

  return cs_grasp_id;
}

//HybridObjectStateSpace::PoseComponent::PoseComponent(const robot_model::JointModelGroup *subgroup,
//                                                     const robot_model::JointModelGroup::KinematicsSolver &k,
//                                                     const moveit_msgs::Grasp& grasp)
//  : subgroup_(subgroup)
//  , kinematics_solver_(k.allocator_(subgroup))
//  , grasp_(grasp)
//{
//
//  state_space_.reset(new ompl::base::SE3StateSpace());
//  state_space_->setName(subgroup_->getName() + "_Workspace");
//  fk_link_.resize(1, kinematics_solver_->getTipFrame());
//  if (!fk_link_[0].empty() && fk_link_[0][0] == '/')
//    fk_link_[0] = fk_link_[0].substr(1);
//}