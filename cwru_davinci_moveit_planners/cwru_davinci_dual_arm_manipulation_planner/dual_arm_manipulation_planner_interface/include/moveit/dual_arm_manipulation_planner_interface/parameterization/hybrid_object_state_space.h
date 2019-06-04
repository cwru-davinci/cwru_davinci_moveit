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

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/DiscreteStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <cwru_davinci_grasp/davinci_simple_grasp_generator.h>

namespace dual_arm_manipulation_planner_interface
{
enum class StateDiff
{
  AllSame,
  ArmDiffGraspAndPoseSame,
  GraspDiffArmAndPoseSame,
  PoseDiffArmAndGraspSame,
  ArmAndGraspDiffPoseSame,
  ArmAndPoseDiffGraspSame,
  GraspAndPoseDiffArmSame,
  AllDiff
};

OMPL_CLASS_FORWARD(HybridObjectStateSpace);

class HybridStateSampler : public ompl::base::StateSampler
{
public:
  HybridStateSampler(const HybridObjectStateSpace *space);

  void sampleUniform(ompl::base::State *state) override;

  void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;

  void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;
private:
  const HybridObjectStateSpace *hyStateSpace_;

  robot_model::RobotModelPtr kmodel_;

  robot_model_loader::RobotModelLoader robot_model_loader_;

//  std::string robot_name_;
};

class HybridObjectStateSpace : public ompl::base::CompoundStateSpace
{
public:

  void printExecutionDuration();
  void resetTimer();

  static std::chrono::duration<double> object_transit_planning_duration_;

  static std::chrono::duration<double> check_motion_duration_;

  static std::chrono::duration<double> validity_checking_duration_;

  static std::chrono::duration<double> interpolation_duration_;

  static std::chrono::duration<double> hand_off_duration_;

  static std::chrono::duration<double> ik_solving_duration_;

  static std::chrono::duration<double> sampling_duration_;

  static std::chrono::duration<double> choose_grasp_duration_;

  static std::chrono::duration<double> compute_ik_duration_;

  static std::chrono::duration<double> collision_checking_duration_;

  static int sampling_num;

  static int validty_check_num;

  static int call_interpolation_num;

  static int check_motion_num;

  static int object_transit_motion_planner_num;

  static int hand_off_planning_num;

  static int hand_off_failed_num;

  class StateType : public ompl::base::CompoundStateSpace::StateType
  {
  public:
    enum
    {
      VALIDITY_KNOWN = 1,
      VALIDITY_TRUE = 4,
      IS_TRANSFER = 8,
      IS_TRANSIT = 16,
      JOINTS_COMPUTED = 256
    };

    StateType() : ompl::base::CompoundStateSpace::StateType(), flags(0)
    {
    }


//    bool isTransfer() const
//    {
//      return flags & IS_TRANSFER;
//    }
//
//    bool isTransit() const
//    {
//      return flags & IS_TRANSIT;
//    }
//
//    void markTransfer() const
//    {
//      flags |= IS_TRANSFER;
//    }
//
//    void markTransit() const
//    {
//      flags |= IS_TRANSIT;
//    }
//
    void markInvalid()
    {
      flags &= ~VALIDITY_TRUE;
      flags |= VALIDITY_KNOWN;
    }

    bool isValidityKnown() const
    {
      return flags & VALIDITY_KNOWN;
    }

    void markValid()
    {
      flags |= (VALIDITY_KNOWN | VALIDITY_TRUE);
    }

    bool isMarkedValid() const
    {
      return flags & VALIDITY_TRUE;
    }

    void setJointsComputed(bool value)
    {
      if (value)
        flags |= JOINTS_COMPUTED;
      else
        flags &= ~JOINTS_COMPUTED;
    }

    bool jointsComputed() const
    {
      return flags & JOINTS_COMPUTED;
    }

    void clearKnownInformation()
    {
      flags = 0;
    }

    /**
     * @brief Get the SE(3) components of the state and allow changing it as well
     * @return
     */
    const ompl::base::SE3StateSpace::StateType &se3State() const
    {
      return *as<ompl::base::SE3StateSpace::StateType>(0);
    }

    ompl::base::SE3StateSpace::StateType &se3State()
    {
      return *as<ompl::base::SE3StateSpace::StateType>(0);
    }

    /**
     * @brief Get the Arm index components of the state and allow changing it as well
     * @return
     */
    const ompl::base::DiscreteStateSpace::StateType &armIndex() const
    {
      return *as<ompl::base::DiscreteStateSpace::StateType>(1);
    }

    /**
     * @brief Get the arm index components of the state and allow changing it as well
     * @return
     */
    ompl::base::DiscreteStateSpace::StateType &armIndex()
    {
      return *as<ompl::base::DiscreteStateSpace::StateType>(1);
    }


    /**
     * @brief Get the grasp index components of the state and allow changing it as well
     * @return
     */
    const ompl::base::DiscreteStateSpace::StateType &graspIndex() const
    {
      return *as<ompl::base::DiscreteStateSpace::StateType>(2);
    }

    /**
     * @brief Get the grasp index components of the state and allow changing it as well
     * @return
     */
    ompl::base::DiscreteStateSpace::StateType &graspIndex()
    {
      return *as<ompl::base::DiscreteStateSpace::StateType>(2);
    }

    const ompl::base::RealVectorStateSpace::StateType &jointVariables() const
    {
      return *as<ompl::base::RealVectorStateSpace::StateType>(3);
    }

    ompl::base::RealVectorStateSpace::StateType &jointVariables()
    {
      return *as<ompl::base::RealVectorStateSpace::StateType>(3);
    }

    void clearJointValues() const
    {
      auto *rstate = as<ompl::base::RealVectorStateSpace::StateType>(3);
      delete[] rstate->values;
      delete rstate;
    }

//    double *values;
//    int tag;
    int flags;
//    double distance;
  };

  HybridObjectStateSpace(int armIndexLowerBound,
                         int armIndexUpperBound,
                         int graspIndexLowerBound,
                         int graspIndexUpperBound,
                         const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps,
                         const std::string &robot_name = "robot_description");

  virtual ~HybridObjectStateSpace()
  {}

  void setSE3Bounds(const ompl::base::RealVectorBounds &bounds);

  void setArmIndexBounds(int lowerBound, int upperBound);

  void setGraspIndexBounds(int lowerBound, int upperBound);

  bool setJointValues(const std::vector<double> &joint_values, StateType *state) const;

  int getJointSpaceDimension() const;

  virtual bool isHybrid() const;

  virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

  virtual void serialize(void *serialization, const ompl::base::State *state) const override;

  virtual ompl::base::State *allocState() const override;

  virtual void freeState(ompl::base::State *state) const override;

  virtual void copyState(ompl::base::State *destination, const ompl::base::State *source) const override;

  virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override;

  virtual void printState(const ompl::base::State *state, std::ostream &out) const override;

  virtual void interpolate(const ompl::base::State *from,
                           const ompl::base::State *to,
                           const double t,
                           ompl::base::State *state) const override;

  virtual void copyToReals(std::vector<double> &reals, const ompl::base::State *source) const override;
//  virtual double getMaximumExtent() const;

  virtual ompl::base::StateSamplerPtr allocDefaultStateSampler() const override;

  virtual ompl::base::StateSamplerPtr allocStateSampler() const override;

  /**
   * @brief Count how many segments of the "longest valid length" fit on the motion from state1 to state2.
   * @param state1
   * @param state2
   * @return
   */
  virtual unsigned int validSegmentCount(const ompl::base::State *state1, const ompl::base::State *state2) const;

//  bool computeStateK(ompl::base::State *state) const;

//  bool discreteGeodesic(const State *from, const State *to, bool interpolate,
//  bool computeStateFK(ompl::base::State *state) const;
//
//  bool computeStateIK(ompl::base::State *state) const;
//
//  virtual void sanityChecks() const;
//
//  virtual void copyToOMPLState(ompl::base::State *state, const robot_state::RobotState &rstate) const;
//
//  virtual void setPlanningVolume(double minX, double maxX, double minY, double maxY, double minZ, double maxZ);
  void se3ToEigen3d(const StateType *state, Eigen::Affine3d &affine3d) const;

  void eigen3dToSE3(StateType *state, const Eigen::Affine3d &affine3d) const;

  StateDiff checkStateDiff(const StateType *state1, const StateType *state2) const;

  std::vector<cwru_davinci_grasp::GraspInfo> possible_grasps_;
protected:

  /** \brief SpaceInformation associated with this space. Required
   * for early collision checking in manifold traversal.
   */
//  ompl::base::SpaceInformation *si_{nullptr};

  /** \brief Step size when traversing the manifold and collision checking. */
  double delta_;

  /** \brief Manifold traversal from x to y is stopped if accumulated
   * distance is greater than d(x,y) times this. Additionally, if d(x,
   * y) is greater than lambda * delta between two points, search is
   * discontinued.
   */
  double lambda_;

  /** \brief Whether setup() has been called. */
  bool setup_{false};

private:

  int chooseSupportArm(const int from_arm_index, const int to_arm_index) const;

  bool findSupportArmAndGrasp(const StateType *from,
                              const StateType *to,
                              StateType *cstate) const;

  bool findValidGrasp(int from_part_id, int to_part_id, StateType *cstate) const;

  int handOffsNum(const int from_arm_index,
                  const int to_arm_index,
                  const int from_part_id,
                  const int to_part_id) const;

  bool computeStateIK(StateType *hystate) const;

  /**
   * @brief Return the first grasp_id which its part_id different than @param from_part_id and @param @to_part_id
   * @param from_part_id
   * @param to_part_id
   * @return
   */
  int chooseGraspPart(int from_part_id, int to_part_id) const;

  robot_model::RobotModelPtr kmodel_;

  robot_model_loader::RobotModelLoader robot_model_loader_;

  std::string robot_name_;

  void initializeIKPlugin();

  bool setFromIK(robot_state::RobotState &rstate,
                 const robot_state::JointModelGroup *arm_joint_group,
                 const std::string &planning_group,
                 const std::string &tip_frame,
                 const Eigen::Affine3d &tip_pose_wrt_world) const;

  boost::shared_ptr<kinematics::KinematicsBase> psm_one_kinematics_solver_;
  boost::shared_ptr<kinematics::KinematicsBase> psm_two_kinematics_solver_;
  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> >
    kinematics_loader_;
//  struct PoseComponent
//  {
//    PoseComponent(const robot_model::JointModelGroup *subgroup,
//                  const robot_model::JointModelGroup::KinematicsSolver &k,
//                  const moveit_msgs::Grasp &grasp);
//
//    bool computeStateFK(StateType *full_state) const;
//
//    bool computeStateIK(StateType *full_state) const;
//
//    bool operator<(const PoseComponent &o) const
//    {
//      return subgroup_->getName() < o.subgroup_->getName();
//    }
//
//    const robot_model::JointModelGroup *subgroup_;
//    boost::shared_ptr<kinematics::KinematicsBase> kinematics_solver_;
//    moveit_msgs::Grasp grasp_;
////    std::vector<unsigned int> bijection_;
//    ompl::base::StateSpacePtr state_space_;
//    std::vector<std::string> fk_link_;
//  };
//
//  PoseComponent object_pose_;
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_STATE_SPACE_H
