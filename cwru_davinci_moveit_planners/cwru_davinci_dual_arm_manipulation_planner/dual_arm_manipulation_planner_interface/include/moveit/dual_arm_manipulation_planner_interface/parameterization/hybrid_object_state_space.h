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

namespace ompl
{
  namespace base
  {
    class HybridObjectStateSpace : public CompoundStateSpace
    {
    public:

      class StateType : public CompoundStateSpace::StateType
      {
      public:
        StateType() : CompoundStateSpace::StateType()
        {
        }

        const SE3StateSpace::StateType & se3State() const
        {
          return *as<SE3StateSpace::StateType>(0);
        }

        SE3StateSpace::StateType & se3State()
        {
          return *as<SE3StateSpace::StateType>(0);
        }

        const DiscreteStateSpace::StateType & armIndex() const
        {
          return *as<DiscreteStateSpace::StateType>(1);
        }

        DiscreteStateSpace::StateType & armIndex()
        {
          return *as<DiscreteStateSpace::StateType>(1);
        }

        const DiscreteStateSpace::StateType & graspIndex() const
        {
          return *as<DiscreteStateSpace::StateType>(2);
        }

        DiscreteStateSpace::StateType & graspIndex()
        {
          return *as<DiscreteStateSpace::StateType>(2);
        }
      };


      HybridObjectStateSpace(int armIndexLowerBound, int armIndexUpperBound, int graspIndexLowerBound,
                             int graspIndexUpperBound);

      virtual ~HybridObjectStateSpace() {}

      void setArmIndexBounds(int lowerBound, int upperBound);

      void setGraspIndexBounds(int lowerBound, int upperBound);

      virtual double distance(const State *state1, const State *state2) const override;

      virtual ompl::base::State* allocState() const override;

      const RealVectorBounds getBounds() const;

    private:
      StateSpacePtr state_space_;
    };
  }
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_OBJECT_STATE_SPACE_H
