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
 * Description:
 */


#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_MODEL_BASED_HYBRID_STATE_SPACE_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_MODEL_BASED_HYBRID_STATE_SPACE_H

#include <ompl/base/StateSpace.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/AttachedCollisionObject.h>


namespace dual_arm_manipulation_planner_interface
{
typedef boost::function<bool(const ompl::base::State* from, const ompl::base::State* to, const double t, ompl::base::State* state)> InterpolationFunction;

typedef boost::function<double(const ompl::base::State* state1, const ompl::base::State* state2)> DistanceFunction;

struct ModelBasedHybridStateSpaceSpecification
{

  ModelBasedHybridStateSpaceSpecification(const robot_model::RobotModelConstPtr &robot_model,
                                          const robot_model::JointModelGroup *jmp,
                                          const moveit_msgs::AttachedCollisionObject &held_object,
                                          const std::vector <moveit_msgs::Grasp> &possible_grasps) : robot_model_(
    robot_model), joint_model_group_(jmg), held_object_(held_object), possible_grasps_(possible_grasps)
  {

  }

  ModelBasedHybridStateSpaceSpecification(const robot_model::RobotModelConstPtr &robot_model,
                                          const std::string &group_name,
                                          const moveit_msgs::AttachedCollisionObject &held_object,
                                          const std::vector <moveit_msgs::Grasp> &possible_grasps)
    : robot_model_(robot_model), joint_model_group_(robot_model_->getJointModelGroup(group_name)),
      held_object_(held_object), possible_grasps_(possible_grasps)
  {

    if (!joint_model_group_)
      throw std::runtime_error("Group '" + group_name + "'  was not found");
  }

  robot_model::RobotModelConstPtr robot_model_;
  const robot_model::JointModelGroup* joint_model_group_;
  robot_model::JointBoundsVector joint_bounds_;

  moveit_msgs::AttachedCollisionObject held_object_;
  std::vector<moveit_msgs::Grasp>& possible_grasps_;
};

class ModelBasedHybridStateSpace : public ompl::base::StateSpace
{
public:
  class StateType : public ompl::base::State
  {
  public:
    enum
    {
      VALIDITY_KNOWN = 1,
      GOAL_DISTANCE_KNOWN = 2,
      VALIDITY_TRUE = 4,
      IS_START_STATE = 8,
      IS_GOAL_STATE = 16
    };

    StateType() : ompl::base::State(), values(NULL), tag(-1), flags(0), distance(0.0)
    {
    }

    void markValid(double d)
    {
      distance = d;
      flags |= GOAL_DISTANCE_KNOWN;
      markValid();
    }

    void markValid()
    {
      flags |= (VALIDITY_KNOWN |= VALIDITY_TRUE);
    }

    void markInvalid(double d)
    {
      distance = d;
      flags |= GOAL_DISTANCE_KNOWN;
      markInvalid();
    }

    void markInvalid()
    {
      flags &= ~VALIDITY_TRUE;
      flags |= VALIDITY_KNOWN;
    }

    bool isValidityKnown() const
    {
      return flags & VALIDITY_KNOWN;
    }

    void clearKnownInformation()
    {
      flags = 0;
    }

    bool isMarkedValid() const
    {
      return flags & VALIDITY_TRUE;
    }

    bool isGoalDistanceKnown() const
    {
      return flags & GOAL_DISTANCE_KNOWN;
    }

    bool isStartState() const
    {
      return flags & IS_START_STATE;
    }

    bool isGoalState() const
    {
      return flags & IS_GOAL_STATE;
    }

    bool isInputState() const
    {
      return flags & (IS_START_STATE | IS_GOAL_STATE);
    }

    void markStartState()
    {
      flags |= IS_START_STATE;
    }

    void markGoalState()
    {
      flags |= IS_GOAL_STATE;
    }

    double* values;
    int tag;
    int flags;
    double distance;
  };

  ModelBasedHybridStateSpace(const ModelBasedHybridStateSpaceSpecification& spec);

  virtual ~ModelBasedHybridStateSpace();

  void setInterpolationFunction(const InterpolationFunction& fun)
  {
    interpolation_function_ = fun;
  }

  void setDistanceFunction(const DistanceFunction& fun)
  {
    distance_function_ = fun;
  }

  virtual ompl::base::State* allocState() const;
};


}


#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_MODEL_BASED_HYBRID_STATE_SPACE_H
