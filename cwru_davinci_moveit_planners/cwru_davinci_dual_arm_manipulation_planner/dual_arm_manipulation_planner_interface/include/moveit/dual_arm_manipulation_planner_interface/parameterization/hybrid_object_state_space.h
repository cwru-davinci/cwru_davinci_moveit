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
 * Description: This hybrid SE3 state space is compound state space of SE3 state space
 * in addition of two discrete state space (Arm index and Grasp index)
 */

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_STATE_SPACE_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_STATE_SPACE_H

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/DiscreteStateSpace.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "moveit/ompl_interface/parameterization/work_space/pose_model_state_space.h"
#include <boost/scoped_ptr.hpp>


namespace dual_arm_manipulation_planner_interface
{
class HybridObjectStateSpace : public ompl::base::CompoundStateSpace
{
public:

  class StateType : public ompl::base::CompoundStateSpace::StateType
  {
  public:
    enum
    {
      JOINTS_COMPUTED = 256,
      POSE_COMPUTED = 512
    };

    StateType() : CompoundStateSpace::StateType(), values(NULL), flags(0)
    {
      flags |= JOINTS_COMPUTED;
    }

    bool jointsComputed() const
    {
      return flags & JOINTS_COMPUTED;
    }

    bool poseComputed() const
    {
      return flags & POSE_COMPUTED;
    }

    void setJointsComputed(bool value)
    {
      if (value)
        flags |= JOINTS_COMPUTED;
      else
        flags &= ~JOINTS_COMPUTED;
    }

    void setPoseComputed(bool value)
    {
      if (value)
        flags |= POSE_COMPUTED;
      else
        flags &= ~POSE_COMPUTED;
    }

    const ompl::base::SE3StateSpace::StateType &se3State() const
    {
      return *as<ompl::base::SE3StateSpace::StateType>(0);
    }

    ompl::base::SE3StateSpace::StateType &se3State()
    {
      return *as<ompl::base::SE3StateSpace::StateType>(0);
    }

    const ompl::base::DiscreteStateSpace::StateType &armIndex() const
    {
      return *as<ompl::base::DiscreteStateSpace::StateType>(1);
    }

    ompl::base::DiscreteStateSpace::StateType &armIndex()
    {
      return *as<ompl::base::DiscreteStateSpace::StateType>(1);
    }

    const ompl::base::DiscreteStateSpace::StateType &graspIndex() const
    {
      return *as<ompl::base::DiscreteStateSpace::StateType>(2);
    }

    ompl::base::DiscreteStateSpace::StateType &graspIndex()
    {
      return *as<ompl::base::DiscreteStateSpace::StateType>(2);
    }

    double *values;
//    int tag;
    int flags;
//    double distance;

  };


  HybridObjectStateSpace(int armIndexLowerBound, int armIndexUpperBound, int graspIndexLowerBound,
                         int graspIndexUpperBound, const ompl_interface::ModelBasedStateSpaceSpecification &spec);

  virtual ~HybridObjectStateSpace()
  {}

  void setArmIndexBounds(int lowerBound, int upperBound);

  void setGraspIndexBounds(int lowerBound, int upperBound);

  virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

  virtual ompl::base::State *allocState() const override;

  virtual void freeState(ompl::base::State *state) const;

  virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const;

  virtual void interpolate(const ompl::base::State *from,
                           const ompl::base::State *to,
                           const double t,
                           ompl::base::State *state) const;

  virtual double getMaximumExtent() const;

  virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const;

  bool computeStateFK(ompl::base::State *state) const;

  bool computeStateIK(ompl::base::State *state) const;
//
//  bool computeStateK(ompl::base::State *state) const;
//
//  virtual void setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
//
//  virtual void copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const;
//
//  virtual void sanityChecks() const;

private:
//  struct HybridObjectPoseComponent
//  {
//    HybridObjectPoseComponent(const robot_model::JointModelGroup *subgroup,
//                              const robot_model::JointModelGroup::KinematicsSolver &k,
//                              const std::vector<moveit_msgs::Grasp> &grasp_list);
//
//    bool computeStateFK(StateType *full_state, unsigned int idx) const;
//
//    bool computeStateIK(StateType *full_state, unsigned int idx) const;
//
//    bool operator<(const PoseComponent &o) const
//    {
//      return subgroup_->getName() < o.subgroup_->getName();
//    }
//
//    const robot_model::JointModelGroup *subgroup_;
//    boost::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;
//    std::vector<moveit_msgs::Grasp> grasp_list_;
//    std::vector<unsigned int> bijection_;
//    ompl::base::StateSpacePtr state_space_;
//    std::vector<std::string> fk_link_;
//  };
//
//  HybridObjectPoseComponent object_pose_;

  boost::scoped_ptr<ompl_interface::PoseModelStateSpace> pose_model_ss_;

//  ompl_interface::ModelBasedStateSpaceSpecification spec_;
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_STATE_SPACE_H
