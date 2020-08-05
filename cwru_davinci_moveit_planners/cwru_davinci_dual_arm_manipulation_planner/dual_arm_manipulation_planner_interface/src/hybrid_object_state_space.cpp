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

#include <dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <ompl/base/ValidStateSampler.h>
#include <unordered_set>

using namespace dual_arm_manipulation_planner_interface;
using namespace ompl::base;

HybridStateSampler::HybridStateSampler(const HybridObjectStateSpace* space)
: StateSampler(space),
  hyStateSpace_(space),
  robot_model_loader_("robot_description")
{
  kmodel_ = robot_model_loader_.getModel();

  hyStateSpace_->sampling_duration_ = std::chrono::duration<double>::zero();
}

void HybridStateSampler::sampleUniform(State* state)
{
  auto start = std::chrono::high_resolution_clock::now();
  hyStateSpace_->sampling_num += 1;

  ompl::base::DiscreteStateSampler arm_index_sampler(hyStateSpace_->getSubspace(1).get());
  ompl::base::DiscreteStateSampler grasp_index_sampler(hyStateSpace_->getSubspace(2).get());

  auto* hss = static_cast<HybridObjectStateSpace::StateType*>(state);
  hss->clearKnownInformation();

  arm_index_sampler.sampleUniform(&hss->armIndex());
  grasp_index_sampler.sampleUniform(&hss->graspIndex());

  const robot_state::RobotStatePtr robot_sample_state(new robot_state::RobotState(kmodel_));
  if (!robot_sample_state)
  {
    return;
  }
  robot_sample_state->setToDefaultValues();

  std::string planning_group = (hss->armIndex().value == 1) ? "psm_one" : "psm_two";
  const robot_state::JointModelGroup* selected_joint_model_group = robot_sample_state->getJointModelGroup(planning_group);
  robot_sample_state->setToRandomPositions(selected_joint_model_group);
  robot_sample_state->update();
  robot_sample_state->copyJointGroupPositions(selected_joint_model_group, hss->jointVariables().values);
  hss->setJointsComputed(true);

  const Eigen::Affine3d tool_tip_pose = robot_sample_state->getGlobalLinkTransform(
  selected_joint_model_group->getOnlyOneEndEffectorTip());
  const Eigen::Affine3d grasp_pose = hyStateSpace_->graspTransformations()[hss->graspIndex().value].grasp_pose;
  const Eigen::Affine3d object_pose = tool_tip_pose * grasp_pose;
  hyStateSpace_->eigen3dToSE3(object_pose, hss);

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  hyStateSpace_->sampling_duration_ += elapsed;
}

void HybridStateSampler::sampleUniformNear
(
State* state,
const State* near,
double distance
)
{
  // left blank
}

void HybridStateSampler::sampleGaussian
(
State* state,
const State* mean,
double stdDev
)
{
  // left blank
}

HybridObjectStateSpace::HybridObjectStateSpace
(
const int armIdxLwBd,
const int armIdxUpBd,
const int graspIdxLwBd,
const int graspIdxUpBd,
const std::vector<cwru_davinci_grasp::GraspInfo>& possible_grasps
)
: CompoundStateSpace(),
  m_ArmIdxLwBd(armIdxLwBd),
  m_ArmIdxUpBd(armIdxUpBd),
  m_GraspIdxLwBd(graspIdxLwBd),
  m_GraspIdxUpBd(graspIdxUpBd),
  m_PossibleGrasps(possible_grasps)
{
  setName("HybridObject" + getName());
  type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 10;

  addSubspace(std::make_shared<SE3StateSpace>(), 1.0);  // object pose space
  components_.back()->setName(components_.back()->getName() + ":ObjectPose");

  addSubspace(std::make_shared<DiscreteStateSpace>(m_ArmIdxLwBd, m_ArmIdxUpBd),
              1.0);  // arm index
  components_.back()->setName(components_.back()->getName() + ":ArmIndex");

  addSubspace(std::make_shared<DiscreteStateSpace>(m_GraspIdxLwBd, m_GraspIdxUpBd),
              1.0);  // grasp index
  components_.back()->setName(components_.back()->getName() + ":GraspIndex");

  addSubspace(std::make_shared<RealVectorStateSpace>(6), 1.0);
  components_.back()->setName(components_.back()->getName() + ":JointVariables");
  components_[3]->as<RealVectorStateSpace>()->setBounds(-7, 7);

  lock();
}

HybridObjectStateSpace::HybridObjectStateSpace
(
)
: CompoundStateSpace()
{
  setName("HybridObject" + getName());
  type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 10;

  addSubspace(std::make_shared<SE3StateSpace>(), 1.0);  // object pose space
  components_.back()->setName(components_.back()->getName() + ":ObjectPose");

  addSubspace(std::make_shared<DiscreteStateSpace>(0, 0), 1.0);  // arm index
  components_.back()->setName(components_.back()->getName() + ":ArmIndex");

  addSubspace(std::make_shared<DiscreteStateSpace>(0, 0), 1.0);  // grasp index
  components_.back()->setName(components_.back()->getName() + ":GraspIndex");

  addSubspace(std::make_shared<RealVectorStateSpace>(6), 1.0);
  components_.back()->setName(components_.back()->getName() + ":JointVariables");
  components_[3]->as<RealVectorStateSpace>()->setBounds(-7, 7);

  lock();
}

HybridObjectStateSpace::HybridObjectStateSpace
(
const std::vector<cwru_davinci_grasp::GraspInfo>& possible_grasps
)
: CompoundStateSpace(),
  m_PossibleGrasps(possible_grasps)
{
  setName("HybridObject" + getName());
  type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 10;

  addSubspace(std::make_shared<SE3StateSpace>(), 1.0);  // object pose space
  components_.back()->setName(components_.back()->getName() + ":ObjectPose");

  addSubspace(std::make_shared<DiscreteStateSpace>(0, 0), 1.0);  // arm index
  components_.back()->setName(components_.back()->getName() + ":ArmIndex");

  if (!m_PossibleGrasps.empty())
  {
    m_GraspIdxLwBd = 0;
    m_GraspIdxUpBd = static_cast<int>(m_PossibleGrasps.size() - 1);
  }
  addSubspace(std::make_shared<DiscreteStateSpace>(m_GraspIdxLwBd, m_GraspIdxUpBd), 1.0);  // grasp index
  components_.back()->setName(components_.back()->getName() + ":GraspIndex");

  addSubspace(std::make_shared<RealVectorStateSpace>(6), 1.0);
  components_.back()->setName(components_.back()->getName() + ":JointVariables");
  components_[3]->as<RealVectorStateSpace>()->setBounds(-7, 7);

  lock();
}

std::chrono::duration<double> HybridObjectStateSpace::object_transit_planning_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::check_motion_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::validity_checking_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::interpolation_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::hand_off_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::ik_solving_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::sampling_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::choose_grasp_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::compute_ik_duration_ = std::chrono::duration<double>::zero();
std::chrono::duration<double> HybridObjectStateSpace::collision_checking_duration_ = std::chrono::duration<double>::zero();

int HybridObjectStateSpace::sampling_num = 0;
int HybridObjectStateSpace::validty_check_num = 0;
int HybridObjectStateSpace::call_interpolation_num = 0;
int HybridObjectStateSpace::check_motion_num = 0;
int HybridObjectStateSpace::object_transit_motion_planner_num = 0;
int HybridObjectStateSpace::hand_off_planning_num = 0;
int HybridObjectStateSpace::hand_off_failed_num = 0;

void HybridObjectStateSpace::resetTimer()
{
  HybridObjectStateSpace::object_transit_planning_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::check_motion_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::validity_checking_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::interpolation_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::hand_off_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::ik_solving_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::sampling_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::choose_grasp_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::compute_ik_duration_ = std::chrono::duration<double>::zero();
  HybridObjectStateSpace::collision_checking_duration_ = std::chrono::duration<double>::zero();

  HybridObjectStateSpace::sampling_num = 0;
  HybridObjectStateSpace::validty_check_num = 0;
  HybridObjectStateSpace::call_interpolation_num = 0;
  HybridObjectStateSpace::check_motion_num = 0;
  HybridObjectStateSpace::object_transit_motion_planner_num = 0;
  HybridObjectStateSpace::hand_off_planning_num = 0;
  HybridObjectStateSpace::hand_off_failed_num = 0;
}

void HybridObjectStateSpace::printExecutionDuration(
double* total_time,
bool verbose
)
{

  if (verbose)
  {
    std::cout << "Sampling Elapsed duration and times of called: " << sampling_duration_.count() << "s and "
              << sampling_num << "\n"
              << "Validity Check Elapsed duration and times of called: " << validity_checking_duration_.count()
              << "s and " << validty_check_num << "\n"
              << "Interpolation Elapsed duration: " << interpolation_duration_.count() << "s\n"
              << "Interplate function has been called: " << call_interpolation_num << " times\n"
              << "Compute IK Elapsed duration in Interpolation function: " << compute_ik_duration_.count() << "s\n"
              << "Check Motion Elapsed duration: " << check_motion_duration_.count() << "s\n"
              << "Check Motion has been called: " << check_motion_num << " times\n"
              << "Object Transit Motion Planning Elapsed duration: " << object_transit_planning_duration_.count()
              << "s\n"
              << "Object Transit Motion Planner has been called: " << object_transit_motion_planner_num << " times\n"
              << "Handoff Elapsed duration: " << hand_off_duration_.count() << "s\n"
              << "Handoff Planning has been called: " << hand_off_planning_num << " times\n"
              << "Handoff Planning Failed times: " << hand_off_failed_num << "\n"
              << "IK sovling Elapsed Time in Check Motion: " << ik_solving_duration_.count() << "s\n"
              << "Local Planner Collision Check Elapsed Time: " << collision_checking_duration_.count() << "s\n"
              << "Choose Grasp Function Elapsed Time: " << choose_grasp_duration_.count() << std::endl;
  }

  std::chrono::duration<double> total_time_chro =
  sampling_duration_ + validity_checking_duration_ + interpolation_duration_ + check_motion_duration_;
  if (total_time)
  {
    *total_time = total_time_chro.count();
  }

  std::cout << "Total Time is: " << total_time_chro.count() << "s" << std::endl;
}

void HybridObjectStateSpace::setSE3Bounds(const RealVectorBounds& bounds)
{
  components_[0]->as<SE3StateSpace>()->setBounds(bounds);
}

void HybridObjectStateSpace::setSE3Bounds
(
const double se3BoundXAxisMin,
const double se3BoundXAxisMax,
const double se3BoundYAxisMin,
const double se3BoundYAxisMax,
const double se3BoundZAxisMin,
const double se3BoundZAxisMax
)
{
  RealVectorBounds se3XYZBounds(3);
  se3XYZBounds.setLow(0, se3BoundXAxisMin);
  se3XYZBounds.setHigh(0, se3BoundXAxisMax);
  se3XYZBounds.setLow(1, se3BoundYAxisMin);
  se3XYZBounds.setHigh(1, se3BoundYAxisMax);
  se3XYZBounds.setLow(2, se3BoundZAxisMin);
  se3XYZBounds.setHigh(2, se3BoundZAxisMax);
  setSE3Bounds(se3XYZBounds);
}

void HybridObjectStateSpace::setArmIndexBounds
(
int lowerBound,
int upperBound
)
{
  m_ArmIdxLwBd = lowerBound;
  m_ArmIdxUpBd = upperBound;
  components_[1]->as<DiscreteStateSpace>()->setBounds(m_ArmIdxLwBd, m_ArmIdxUpBd);
}

void HybridObjectStateSpace::setGraspIndexBounds
(
int lowerBound,
int upperBound
)
{
  m_GraspIdxLwBd = lowerBound;
  m_GraspIdxUpBd = upperBound;
  components_[2]->as<DiscreteStateSpace>()->setBounds(m_GraspIdxLwBd, m_GraspIdxUpBd);
}

bool HybridObjectStateSpace::setJointValues
(
const std::vector<double>& joint_values,
StateType* state
) const
{
  ompl::base::RealVectorStateSpace::StateType& joint_variable = state->jointVariables();
  int joint_space_size = components_[3]->as<RealVectorStateSpace>()->getDimension();
  if (joint_values.size() != joint_space_size)
  {
    printf("Robot's JointModelGroup Variables Dimension is NOT %d \n", joint_space_size);
    return false;
  }

  for (std::size_t i = 0; i < joint_space_size; ++i)
  {
    joint_variable[i] = joint_values[i];
  }

  return true;
}

void HybridObjectStateSpace::copyJointValues
(
const StateType* state,
std::vector<double>& joint_values
) const
{
  const ompl::base::RealVectorStateSpace::StateType& joint_variable = state->jointVariables();
  int joint_space_size = components_[3]->as<RealVectorStateSpace>()->getDimension();
  joint_values.clear();
  joint_values.resize(joint_space_size);

  for (std::size_t i = 0; i < joint_space_size; ++i)
  {
    joint_values[i] = joint_variable[i];
  }
}

int HybridObjectStateSpace::getJointSpaceDimension() const
{
  return components_[3]->as<RealVectorStateSpace>()->getDimension();
}

bool HybridObjectStateSpace::isHybrid() const
{
  return true;
}

double HybridObjectStateSpace::distance
(
const State* state1,
const State* state2
) const
{
  const auto* hs1 = static_cast<const StateType*>(state1);
  const auto* hs2 = static_cast<const StateType*>(state2);

  double se3_dist = components_[0]->distance(hs1->components[0], hs2->components[0]);

  const int s1_arm_index = hs1->armIndex().value;
  const int s2_arm_index = hs2->armIndex().value;

  const int s1_grasp_index = hs1->graspIndex().value;
  const int s2_grasp_index = hs2->graspIndex().value;

  const int s1_part_id = m_PossibleGrasps[s1_grasp_index].part_id;
  const int s2_part_id = m_PossibleGrasps[s2_grasp_index].part_id;

  int num_handoff = handOffsNum(s1_arm_index,
                                s1_grasp_index,
                                s1_part_id,
                                s2_arm_index,
                                s2_grasp_index,
                                s2_part_id);

  int scale = 100;
  double total_dist = scale * num_handoff + se3_dist;

  return total_dist;
}

void HybridObjectStateSpace::serialize
(
void* serialization,
const State* state
) const
{
  const auto* hystate = static_cast<const StateType*>(state);
  unsigned int l = 0;
  for (unsigned int i = 0; i < componentCount_; ++i)
  {
    components_[i]->serialize(reinterpret_cast<char*>(serialization) + l, hystate->components[i]);
    l += components_[i]->getSerializationLength();
  }
}

State* HybridObjectStateSpace::allocState() const
{
  auto* state = new StateType();
  ompl::base::CompoundStateSpace::allocStateComponents(state);
  return static_cast<ompl::base::State*>(state);
}

void HybridObjectStateSpace::freeState(State* state) const
{
  ompl::base::CompoundStateSpace::freeState(state);
}

bool HybridObjectStateSpace::satisfiesBounds(const State* state) const
{
  const auto* cstate = static_cast<const StateType*>(state);
  for (unsigned int i = 0; i < componentCount_ - 1; ++i)
  {
    if (!components_[i]->satisfiesBounds(cstate->components[i]))
    {
      return false;
    }
  }

  return true;
}

void HybridObjectStateSpace::copyState
(
State* destination,
const State* source
) const
{
  ompl::base::CompoundStateSpace::copyState(destination, source);
  destination->as<StateType>()->flags = source->as<StateType>()->flags;
}

bool HybridObjectStateSpace::equalStates
(
const State* state1,
const State* state2
) const
{
  const auto* hs1 = static_cast<const StateType*>(state1);
  const auto* hs2 = static_cast<const StateType*>(state2);
  bool is_se3_equal = components_[0]->equalStates(hs1->components[0], hs2->components[0]);
  bool is_armid_equal = components_[1]->equalStates(hs1->components[1], hs2->components[1]);
  bool is_graspid_equal = components_[2]->equalStates(hs1->components[2], hs2->components[2]);

  if (is_se3_equal && is_armid_equal && is_graspid_equal)
  {
    return true;
  }

  return false;
}

void HybridObjectStateSpace::printState
(
const State* state,
std::ostream& out
) const
{
  ompl::base::CompoundStateSpace::printState(state, out);
}

StateSamplerPtr HybridObjectStateSpace::allocDefaultStateSampler() const
{
  return std::make_shared<HybridStateSampler>(this);
}

StateSamplerPtr HybridObjectStateSpace::allocStateSampler() const
{
  return std::make_shared<HybridStateSampler>(this);
}

unsigned int HybridObjectStateSpace::validSegmentCount
(
const State* state1,
const State* state2
) const
{
  const auto* hs1 = static_cast<const StateType*>(state1);
  const auto* hs2 = static_cast<const StateType*>(state2);

  const int s1_arm_index = hs1->armIndex().value;
  const int s2_arm_index = hs2->armIndex().value;

  const int s1_grasp_index = hs1->graspIndex().value;
  const int s2_grasp_index = hs2->graspIndex().value;

  const int s1_part_id = m_PossibleGrasps[s1_grasp_index].part_id;
  const int s2_part_id = m_PossibleGrasps[s2_grasp_index].part_id;

  unsigned int se3_count = components_[0]->validSegmentCount(hs1->components[0], hs2->components[0]);


  if (s1_arm_index == s2_arm_index && s1_grasp_index == s2_grasp_index)
  {
    return se3_count;
  }
  else
  {
    unsigned int handoff_count = handOffsNum(s1_arm_index,
                                             s1_grasp_index,
                                             s1_part_id,
                                             s2_arm_index,
                                             s2_grasp_index,
                                             s2_part_id);
    return handoff_count + se3_count;
  }
}


void HybridObjectStateSpace::interpolate
(
const State* from,
const State* to,
const double t,
State* state
) const
{
  auto start = std::chrono::high_resolution_clock::now();
  auto* cstate = static_cast<StateType*>(state);
  cstate->clearKnownInformation();

  const auto* hys_from = static_cast<const StateType*>(from);
  const auto* hys_to = static_cast<const StateType*>(to);

  if (components_[0]->distance(hys_from->components[0], hys_to->components[0]) < 0.001)
  {
    return;
  }

  ompl::RNG randNum;
  double random_num = randNum.gaussian01();
  bool bound = 0.40;
  bool within_1Sd = (random_num >= -bound && random_num <= bound) ? true : false;
  bool do_transit = within_1Sd ? true : false;

  if (do_transit)
  {
    components_[0]->interpolate(hys_from->components[0], hys_to->components[0], t, cstate->components[0]);
    components_[1]->copyState(cstate->components[1], hys_from->components[1]);
    components_[2]->copyState(cstate->components[2], hys_from->components[2]);
    components_[3]->copyState(cstate->components[3], hys_from->components[3]);
    return;
  }

  switch (checkStateDiff(hys_from, hys_to))
  {
    case StateDiff::AllSame:
      copyState(cstate, hys_to);
      break;
    case StateDiff::ArmDiffGraspAndPoseSame:
      components_[0]->copyState(cstate->components[0], hys_from->components[0]);
      interpolateGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::GraspDiffArmAndPoseSame:
      components_[0]->copyState(cstate->components[0], hys_from->components[0]);
      interpolateGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::PoseDiffArmAndGraspSame:
      components_[0]->interpolate(hys_from->components[0], hys_to->components[0], t, cstate->components[0]);
      components_[1]->copyState(cstate->components[1], hys_from->components[1]);
      components_[2]->copyState(cstate->components[2], hys_from->components[2]);
      components_[3]->copyState(cstate->components[3], hys_from->components[3]);
      break;
    case StateDiff::ArmAndGraspDiffPoseSame:
      components_[0]->copyState(cstate->components[0], hys_from->components[0]);
      interpolateGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::ArmAndPoseDiffGraspSame:
      components_[0]->copyState(cstate->components[0], hys_from->components[0]);
      interpolateGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::GraspAndPoseDiffArmSame:
      components_[0]->copyState(cstate->components[0], hys_from->components[0]);
      interpolateGrasp(hys_from, hys_to, cstate);
      break;
    case StateDiff::AllDiff:
      components_[0]->copyState(cstate->components[0], hys_from->components[0]);
      interpolateGrasp(hys_from, hys_to, cstate);
      break;
    default:
      // should not be there
      break;
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  interpolation_duration_ += elapsed;
  call_interpolation_num += 1;
}

void HybridObjectStateSpace::copyToReals
(
std::vector<double>& reals,
const State* source
) const
{
  auto* hysource = static_cast<const StateType*>(source);
  components_[0]->copyToReals(reals, hysource->components[0]);
  reals.push_back(hysource->armIndex().value);
  reals.push_back(hysource->graspIndex().value);
  std::vector<double> jointValues;
  components_[3]->copyToReals(jointValues, hysource->components[3]);
  reals.insert(reals.end(), jointValues.begin(), jointValues.end());
}

void HybridObjectStateSpace::se3ToEigen3d
(
const StateType* state,
Eigen::Affine3d& affine3d
) const
{
  affine3d.translation() << state->se3State().getX(), state->se3State().getY(), state->se3State().getZ();

  affine3d.linear() = Eigen::Quaterniond(state->se3State().rotation().w,
                                         state->se3State().rotation().x,
                                         state->se3State().rotation().y,
                                         state->se3State().rotation().z).toRotationMatrix();
}

void HybridObjectStateSpace::eigen3dToSE3
(
const Eigen::Affine3d& affine3d,
StateType* state
) const
{
  state->se3State().setXYZ(affine3d.translation().x(), affine3d.translation().y(), affine3d.translation().z());

  Eigen::Quaterniond qua(affine3d.linear());
  state->se3State().rotation().w = qua.w();
  state->se3State().rotation().x = qua.x();
  state->se3State().rotation().y = qua.y();
  state->se3State().rotation().z = qua.z();
}

StateDiff HybridObjectStateSpace::checkStateDiff
(
const StateType* state1,
const StateType* state2
) const
{
  bool same_pose = components_[0]->equalStates(state1->components[0], state2->components[0]);
  bool same_arm = components_[1]->equalStates(state1->components[1], state2->components[1]);
  bool same_grasp = components_[2]->equalStates(state1->components[2], state2->components[2]);

  if (same_arm)
  {
    if (same_grasp)
    {
      return same_pose ? StateDiff::AllSame : StateDiff::PoseDiffArmAndGraspSame;
    }
    else
    {
      return same_pose ? StateDiff::GraspDiffArmAndPoseSame : StateDiff::GraspAndPoseDiffArmSame;
    }
  }
  else
  {
    if (same_grasp)
    {
      return same_pose ? StateDiff::ArmDiffGraspAndPoseSame : StateDiff::ArmAndPoseDiffGraspSame;
    }
    else
    {
      return same_pose ? StateDiff::ArmAndGraspDiffPoseSame : StateDiff::AllDiff;
    }
  }
}


void HybridObjectStateSpace::interpolateGrasp
(
const StateType* from,
const StateType* to,
StateType* cstate
) const
{
  const int from_arm_index = from->armIndex().value;
  const int to_arm_index = to->armIndex().value;

  const int from_grasp_index = from->graspIndex().value;
  const int to_grasp_index = to->graspIndex().value;

  const int from_part_id = m_PossibleGrasps[from_grasp_index].part_id;
  const int to_part_id = m_PossibleGrasps[to_grasp_index].part_id;

  int num_handoff = handOffsNum(from_arm_index, from_grasp_index, from_part_id, to_arm_index, to_grasp_index,
                                to_part_id);

  switch (num_handoff)
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
    default:
      //  should not be there
      break;
  }
}


int HybridObjectStateSpace::handOffsNum
(
const int from_arm_index,
const int from_grasp_index,
const int from_part_id,
const int to_arm_index,
const int to_grasp_index,
const int to_part_id
) const
{
  if (from_arm_index != to_arm_index)
  {
    return ((from_part_id != to_part_id) ? 1 : 3);
  }
  else  // from_arm_index == to_arm_index
  {
    return ((from_grasp_index != to_grasp_index) ? 2 : 0);
  }
}

int HybridObjectStateSpace::chooseSupportArm
(
const int from_arm_index,
const int to_arm_index
) const
{
  if ((from_arm_index & to_arm_index) == 1)
  {
    return 2;
  }
  else if ((from_arm_index & to_arm_index) == 2)
  {
    return 1;
  }
}

int HybridObjectStateSpace::chooseGraspPart
(
int from_part_id,
int to_part_id
) const
{
  ompl::RNG randNumGenerator;
  int cs_grasp_id;

  std::unordered_set<int> invalid_grasp_list;

  int stop_it_num = m_GraspIdxUpBd + 1;
  while (invalid_grasp_list.size() < stop_it_num)
  {
    cs_grasp_id = randNumGenerator.uniformInt(m_GraspIdxLwBd, m_GraspIdxUpBd);
    if (invalid_grasp_list.find(cs_grasp_id) == invalid_grasp_list.end())  // element is not in the container
    {
      int cs_grasp_part = m_PossibleGrasps[cs_grasp_id].part_id;
      if ((cs_grasp_part != from_part_id) && (cs_grasp_part != to_part_id))  // if failed insert to invalid_grasp_list
      {
        return cs_grasp_id;
      }
      else
      {
        invalid_grasp_list.insert(cs_grasp_id);
      }
    }
  }

  cs_grasp_id = m_GraspIdxUpBd + 1; // should not be there
  return cs_grasp_id;
}
