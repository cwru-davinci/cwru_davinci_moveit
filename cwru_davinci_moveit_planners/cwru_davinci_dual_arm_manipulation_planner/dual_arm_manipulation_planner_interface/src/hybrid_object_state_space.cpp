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
#include <ompl/base/ValidStateSampler.h>
#include <unordered_set>

using namespace dual_arm_manipulation_planner_interface;
using namespace ompl::base;

HybridStateSampler::HybridStateSampler(const HybridObjectStateSpace *space) :
  StateSampler(space), hyStateSpace_(space), robot_model_loader_("robot_description")
{
  kmodel_ = robot_model_loader_.getModel();

  hyStateSpace_->sampling_duration_ = std::chrono::duration<double>::zero();
}

void HybridStateSampler::sampleUniform(State *state)
{
  auto start = std::chrono::high_resolution_clock::now();
  hyStateSpace_->sampling_num += 1;

  ompl::base::DiscreteStateSampler arm_index_sampler(hyStateSpace_->getSubspace(1).get());
  ompl::base::DiscreteStateSampler grasp_index_sampler(hyStateSpace_->getSubspace(2).get());

  auto *hss = static_cast<HybridObjectStateSpace::StateType *>(state);
  hss->clearKnownInformation();

  arm_index_sampler.sampleUniform(&hss->armIndex());
  grasp_index_sampler.sampleUniform(&hss->graspIndex());

  robot_state::RobotStatePtr robot_sample_state;
  robot_sample_state.reset(new robot_state::RobotState(kmodel_));

  std::string planning_group = (hss->armIndex().value == 1) ? "psm_one" : "psm_two";
  const robot_state::JointModelGroup* selected_joint_model_group = robot_sample_state->getJointModelGroup(planning_group);
  robot_sample_state->setToRandomPositions(selected_joint_model_group);
  std::vector<double> joint_values;
  robot_sample_state->copyJointGroupPositions(selected_joint_model_group, hss->jointVariables().values);
  hss->setJointsComputed(true);

  const Eigen::Affine3d tool_tip_pose = robot_sample_state->getGlobalLinkTransform(
    selected_joint_model_group->getOnlyOneEndEffectorTip());
  const Eigen::Affine3d grasp_pose = hyStateSpace_->possible_grasps_[hss->graspIndex().value].grasp_pose;
  const Eigen::Affine3d object_pose = tool_tip_pose * grasp_pose;
  hyStateSpace_->eigen3dToSE3(hss, object_pose);

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  hyStateSpace_->sampling_duration_ += elapsed;
}

void HybridStateSampler::sampleUniformNear(State *state, const State *near, double distance)
{
  // left blank
}

void HybridStateSampler::sampleGaussian(State *state, const State *mean, double stdDev)
{
  // left blank
}

HybridObjectStateSpace::HybridObjectStateSpace(int arm_idx_lw_bd,
                                               int arm_idx_up_bd,
                                               int grasp_idx_lw_bd,
                                               int grasp_idx_up_bd,
                                               const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
                                               const std::string &robot_name)
  : CompoundStateSpace(),
    possible_grasps_(possible_grasps),
    robot_model_loader_(robot_name),
    arm_idx_lw_bd_(arm_idx_lw_bd),
    arm_idx_up_bd_(arm_idx_up_bd),
    grasp_idx_lw_bd_(grasp_idx_lw_bd),
    grasp_idx_up_bd_(grasp_idx_up_bd)

{
//  initializeIKPlugin();

  setName("HybridObject" + getName());
  type_ = ompl::base::STATE_SPACE_TYPE_COUNT + 10;

  addSubspace(std::make_shared<SE3StateSpace>(), 1.0);  // object pose space
  components_.back()->setName(components_.back()->getName() + ":ObjectPose");

  addSubspace(std::make_shared<DiscreteStateSpace>(arm_idx_lw_bd_, arm_idx_up_bd_),
              1.0);  // arm index
  components_.back()->setName(components_.back()->getName() + ":ArmIndex");

  addSubspace(std::make_shared<DiscreteStateSpace>(grasp_idx_lw_bd_, grasp_idx_up_bd_),
              1.0);  // grasp index
  components_.back()->setName(components_.back()->getName() + ":GraspIndex");

  addSubspace(std::make_shared<RealVectorStateSpace>(6), 1.0);
  components_.back()->setName(components_.back()->getName() + ":JointVariables");
  components_[3]->as<RealVectorStateSpace>()->setBounds(-7, 7);

  kmodel_ = robot_model_loader_.getModel();

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

void HybridObjectStateSpace::printExecutionDuration(double* total_time, bool verbose)
{

  if(verbose)
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

  std::chrono::duration<double> total_time_chro =
    sampling_duration_ + validity_checking_duration_ + interpolation_duration_ + check_motion_duration_;
  if(total_time)
    *total_time = total_time_chro.count();
  std::cout << "Total Time is: " << total_time_chro.count() << "s" << std::endl;
}

void HybridObjectStateSpace::setSE3Bounds(const RealVectorBounds &bounds)
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

bool HybridObjectStateSpace::setJointValues(const std::vector<double> &joint_values, StateType *state) const
{

  ompl::base::RealVectorStateSpace::StateType &joint_variable = state->jointVariables();
  int joint_space_size = components_[3]->as<RealVectorStateSpace>()->getDimension();
  if(joint_values.size() != joint_space_size)
  {
    printf("Robot's JointModelGroup Variables Dimension is NOT %d", joint_space_size);
    return false;
  }

  for(int i = 0; i < joint_space_size; i++)
  {
    joint_variable[i] = joint_values[i];
  }

  return true;
}

int HybridObjectStateSpace::getJointSpaceDimension() const
{
  return components_[3]->as<RealVectorStateSpace>()->getDimension();
}

bool HybridObjectStateSpace::isHybrid() const
{
  return true;
}

double HybridObjectStateSpace::distance(const State *state1, const State *state2) const
{
  const auto *hs1 = static_cast<const StateType *>(state1);
  const auto *hs2 = static_cast<const StateType *>(state2);

  double se3_dist = components_[0]->distance(hs1->components[0], hs2->components[0]);

  const int s1_arm_index = hs1->armIndex().value;
  const int s2_arm_index = hs2->armIndex().value;

  const int s1_grasp_index = hs1->graspIndex().value;
  const int s2_grasp_index = hs2->graspIndex().value;

  const int s1_part_id = possible_grasps_[s1_grasp_index].part_id;
  const int s2_part_id = possible_grasps_[s2_grasp_index].part_id;

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

void HybridObjectStateSpace::serialize(void *serialization, const State *state) const
{
  const auto *hystate = static_cast<const StateType*>(state);
  unsigned int l = 0;
  for (unsigned int i = 0; i < componentCount_ - 1; ++i)
  {
    components_[i]->serialize(reinterpret_cast<char *>(serialization) + l, hystate->components[i]);
    l += components_[i]->getSerializationLength();
  }
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

bool HybridObjectStateSpace::satisfiesBounds(const State *state) const
{
  const auto *cstate = static_cast<const StateType *>(state);
  for (unsigned int i = 0; i < componentCount_ - 1; ++i)
    if (!components_[i]->satisfiesBounds(cstate->components[i]))
      return false;
  return true;
}

void HybridObjectStateSpace::copyState(State *destination, const State *source) const
{
  ompl::base::CompoundStateSpace::copyState(destination, source);
  destination->as<StateType>()->flags = source->as<StateType>()->flags;
}

bool HybridObjectStateSpace::equalStates(const State *state1, const State *state2) const
{
  const auto *hs1 = static_cast<const StateType *>(state1);
  const auto *hs2 = static_cast<const StateType *>(state2);
  bool is_se3_equal = components_[0]->equalStates(hs1->components[0], hs2->components[0]);
  bool is_armid_equal = components_[1]->equalStates(hs1->components[1], hs2->components[1]);
  bool is_graspid_equal = components_[2]->equalStates(hs1->components[2], hs2->components[2]);
  if(is_se3_equal && is_armid_equal && is_graspid_equal)
    return true;
  return false;
}

void HybridObjectStateSpace::printState(const State *state, std::ostream &out) const
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

//ValidStateSamplerPtr HybridObjectStateSpace::allocMyValidStateSampler(const SpaceInformation *si)
//{
//  return std::make_shared<HybridValidStateSampler>(si);
//}

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


void HybridObjectStateSpace::interpolate(const State *from,
                                         const State *to,
                                         const double t,
                                         State *state) const
{
  auto start = std::chrono::high_resolution_clock::now();
  auto *cstate = static_cast<StateType *>(state);
  cstate->clearKnownInformation();

  const auto *hys_from = static_cast<const StateType *>(from);
  const auto *hys_to = static_cast<const StateType *>(to);

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
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  interpolation_duration_ += elapsed;
  call_interpolation_num += 1;
}

void HybridObjectStateSpace::copyToReals(std::vector<double> &reals, const State *source) const
{
  auto *hysource = static_cast<const StateType *>(source);
  components_[0]->copyToReals(reals, hysource->components[0]);
  reals.push_back(hysource->armIndex().value);
  reals.push_back(hysource->graspIndex().value);
}

void HybridObjectStateSpace::se3ToEigen3d(const StateType *state, Eigen::Affine3d& affine3d) const
{
  affine3d.translation() << state->se3State().getX(), state->se3State().getY(), state->se3State().getZ();

  Eigen::Matrix3d Re(Eigen::Quaterniond(state->se3State().rotation().w,
                                        state->se3State().rotation().x,
                                        state->se3State().rotation().y,
                                        state->se3State().rotation().z));
  affine3d.linear() = Re;
}

void HybridObjectStateSpace::eigen3dToSE3(StateType *state, const Eigen::Affine3d& affine3d) const
{
  state->se3State().setXYZ(affine3d.translation().x(), affine3d.translation().y(), affine3d.translation().z());

  Eigen::AngleAxisd angle_ax(affine3d.linear());
  Eigen::Vector3d axis = angle_ax.axis();
  axis.normalize();

  state->se3State().rotation().setAxisAngle(axis(0),
                                            axis(1),
                                            axis(2),
                                            angle_ax.angle());
//  printState(state, std::cout);

//  geometry_msgs::Quaternion m;
//  tf::quaternionEigenToMsg(Eigen::Quaterniond(affine3d.linear()), m);
//  state->se3State().rotation().x = m.x;
//  state->se3State().rotation().y = m.y;
//  state->se3State().rotation().z = m.z;
//  state->se3State().rotation().w = m.w;

//  printState(state, std::cout);
}

StateDiff HybridObjectStateSpace::checkStateDiff(const StateType *state1, const StateType *state2) const
{
  bool same_pose = components_[0]->equalStates(state1->components[0], state2->components[0]);
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


void HybridObjectStateSpace::interpolateGrasp(const StateType *from,
                                              const StateType *to,
                                              StateType *cstate) const
{
  const int from_arm_index = from->armIndex().value;
  const int to_arm_index = to->armIndex().value;

  const int from_grasp_index = from->graspIndex().value;
  const int to_grasp_index = to->graspIndex().value;

  const int from_part_id = possible_grasps_[from_grasp_index].part_id;
  const int to_part_id = possible_grasps_[to_grasp_index].part_id;

  int num_handoff = handOffsNum(from_arm_index,from_grasp_index, from_part_id, to_arm_index, to_grasp_index, to_part_id);

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
  }
}


int HybridObjectStateSpace::handOffsNum(const int from_arm_index,
                                        const int from_grasp_index,
                                        const int from_part_id,
                                        const int to_arm_index,
                                        const int to_grasp_index,
                                        const int to_part_id) const
{
  int num_handoff = 0;

  if (from_arm_index != to_arm_index)
  {
    if (from_part_id != to_part_id)
      num_handoff = 1;
    else
      num_handoff = 3;
  }
  else  // from_arm_index == to_arm_index
  {
    if(from_grasp_index != to_grasp_index)
      num_handoff = 2;
  }

  return num_handoff;
}

int HybridObjectStateSpace::chooseSupportArm(const int from_arm_index, const int to_arm_index) const
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

int HybridObjectStateSpace::chooseGraspPart(int from_part_id, int to_part_id) const
{
  ompl::RNG randNumGenerator;
  int cs_grasp_id = randNumGenerator.uniformInt(grasp_idx_lw_bd_, grasp_idx_up_bd_);

  std::unordered_set<int> invalid_grasp_list;

  int stop_it_num = grasp_idx_up_bd_ + 1;
  while(invalid_grasp_list.size() < stop_it_num)
  {
    if(invalid_grasp_list.find(cs_grasp_id) == invalid_grasp_list.end())  // element is not in the container
    {
      int cs_grasp_part = possible_grasps_[cs_grasp_id].part_id;
      if((cs_grasp_part != from_part_id) && (cs_grasp_part != to_part_id))  // if failed insert to invalid_grasp_list
      {
        return cs_grasp_id;
      }
      else
      {
        invalid_grasp_list.insert(cs_grasp_id);
        cs_grasp_id = randNumGenerator.uniformInt(grasp_idx_lw_bd_, grasp_idx_up_bd_);
      }
    }
  }

  cs_grasp_id = grasp_idx_up_bd_ + 1; // should not be there
  return cs_grasp_id;
}

bool HybridObjectStateSpace::findValidGrasp(int from_part_id, int to_part_id, StateType *cstate) const
{
  auto start = std::chrono::high_resolution_clock::now();

  bool is_found = false;

  std::unordered_set<int> invalid_grasp_list;
  ompl::RNG randNumGenerator;
  int random_range = possible_grasps_.size() - 1;
  int stop_it_num = possible_grasps_.size();

  ROS_INFO("Finding valid grasp transformation \n");
  while(invalid_grasp_list.size() < stop_it_num)  // as long as element in invalid_grasp_list <= possible_grasp size do while
  {
    int random_grasp_index = randNumGenerator.uniformInt(0, random_range);
    if(invalid_grasp_list.find(random_grasp_index) == invalid_grasp_list.end())  // element is not in the container
    {
      int grasp_part = possible_grasps_[random_grasp_index].part_id;
      // first round screen
      if((grasp_part == from_part_id) || (grasp_part == to_part_id))  // if failed insert to invalid_grasp_list
      {
        invalid_grasp_list.insert(random_grasp_index);
        continue;
      }

      // if past first round scree then check validity here
      cstate->graspIndex().value = random_grasp_index;
      if(computeStateIK(cstate))
      {
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        choose_grasp_duration_ += elapsed;
        is_found = true;
        return is_found;
      }
      invalid_grasp_list.insert(random_grasp_index);
//      printf("IK Invalid grasp transformation: %d\n", random_grasp_index);
    }
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  choose_grasp_duration_ += elapsed;
  return is_found;
}

bool HybridObjectStateSpace::computeStateIK(StateType *hystate) const
{

  auto start = std::chrono::high_resolution_clock::now();
  robot_state::RobotStatePtr rstate;
  rstate.reset(new robot_state::RobotState(kmodel_));

  Eigen::Affine3d object_pose;  // object pose w/rt base frame
  this->se3ToEigen3d(hystate, object_pose);

  std::string planning_group = (hystate->armIndex().value == 1) ? "psm_one" : "psm_two";
  Eigen::Affine3d grasp_pose = possible_grasps_[hystate->graspIndex().value].grasp_pose;
  Eigen::Affine3d tool_tip_pose = object_pose * grasp_pose.inverse();

  const robot_state::JointModelGroup *selected_joint_model_group = rstate->getJointModelGroup(planning_group);
  std::size_t attempts = 1;
  double timeout = 0.1;

  bool found_ik = rstate->setFromIK(selected_joint_model_group, tool_tip_pose, attempts, timeout);
//  bool found_ik = setFromIK(*rstate, selected_joint_model_group, planning_group,
//                            selected_joint_model_group->getOnlyOneEndEffectorTip()->getName(), tool_tip_pose);

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  compute_ik_duration_ += elapsed;

  if(found_ik)
  {
    hystate->setJointsComputed(true);
    rstate->copyJointGroupPositions(selected_joint_model_group, hystate->jointVariables().values);
  }

  return found_ik;
}

void HybridObjectStateSpace::initializeIKPlugin()
{
  psm_one_kinematics_solver_ = NULL;
  psm_two_kinematics_solver_ = NULL;
  kinematics_loader_.reset(
    new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
  std::string plugin_name = "davinci_moveit_kinematics/DavinciMoveitKinematicsPlugin";

  psm_one_kinematics_solver_ = kinematics_loader_->createInstance(plugin_name);
  psm_two_kinematics_solver_ = kinematics_loader_->createInstance(plugin_name);

  double search_discretization = 0.025;
  psm_one_kinematics_solver_->initialize(robot_name_, "psm_one", "PSM1_psm_base_link", "PSM1_tool_tip_link", search_discretization);
  psm_two_kinematics_solver_->initialize(robot_name_, "psm_two", "PSM2_psm_base_link", "PSM2_tool_tip_link", search_discretization);
}

bool HybridObjectStateSpace::setFromIK(robot_state::RobotState &rstate,
                                       const robot_state::JointModelGroup *arm_joint_group,
                                       const std::string &planning_group,
                                       const std::string &tip_frame,
                                       const Eigen::Affine3d &tip_pose_wrt_world) const
{
  bool found_ik = false;
  std::string base_frame = (planning_group == "psm_one") ? "PSM1_psm_base_link" : "PSM2_psm_base_link";

  boost::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;
  if(planning_group == "psm_one")
  {
    kinematics_solver_ = psm_one_kinematics_solver_;
    if (kinematics_solver_->getGroupName() != planning_group)
      return found_ik;
  }
  else
  {
    kinematics_solver_ = psm_two_kinematics_solver_;
    if (kinematics_solver_->getGroupName() != planning_group)
      return found_ik;
  }

  std::vector<double> seed(kinematics_solver_->getJointNames().size(), 0.0);
  std::vector<double> solution(kinematics_solver_->getJointNames().size(), 0.0);
  double timeout = 0.2;
  Eigen::Affine3d affine_base_wrt_world = rstate.getFrameTransform(base_frame);
  Eigen::Affine3d affine_tip_pose_wrt_base = affine_base_wrt_world.inverse() * tip_pose_wrt_world;
  geometry_msgs::Pose tip_pose_wrt_base;
  tf::poseEigenToMsg(affine_tip_pose_wrt_base, tip_pose_wrt_base);
  moveit_msgs::MoveItErrorCodes error_code;

  found_ik = kinematics_solver_->searchPositionIK(tip_pose_wrt_base, seed, timeout, solution, error_code);

  if (found_ik)
  {
    rstate.setJointGroupPositions(arm_joint_group, solution);
    rstate.update();
  }
  return found_ik;
}
