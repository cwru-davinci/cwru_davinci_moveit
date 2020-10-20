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
 * Description: This class is inherited from ompl::base::SpaceInformation
 */

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_SPACE_INFORMATION_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_SPACE_INFORMATION_H

#include <utility>

#include "ompl/base/SpaceInformation.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

/** \brief Space information for a constrained state space. Implements
         * more direct for getting motion states. */

namespace dual_arm_manipulation_planner_interface
{

class HybridSpaceInformation : public ompl::base::SpaceInformation
{
public:
  /** \brief Constructor. Sets the instance of the state space to plan with. */
  HybridSpaceInformation(StateSpacePtr space) : SpaceInformation (std::move(space))
  {
    stateSpace_->as<HybridObjectStateSpace>()->setSpaceInformation(this);
//    setValidStateSamplerAllocator([](const SpaceInformation *si) -> std::shared_ptr<ValidStateSampler>
//                                  {
//                                    return std::make_shared<ConstrainedValidStateSampler>(si);
//                                  });
  }

  /** \brief Get \e count states that make up a motion between \e s1
      and \e s2. Returns the number of states that were added to \e
      states. Uses the constrained state space's manifold traversal
      method to obtain states. Will always allocate states.

      Otherwise, fewer states can be returned.
      \param s1 the start state of the considered motion
      \param s2 the end state of the considered motion
      \param states the computed set of states along the specified motion
      \param count is currently ignored
      \param endpoints flag indicating whether \e s1 and \e s2 are to be included in states
      \param alloc is currently ignored */
  unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State *> &states,
                               unsigned int count, bool endpoints, bool alloc) const override
  {
    bool success = stateSpace_->as<HybridObjectStateSpace>()->discreteGeodesic(s1, s2, true, &states);

    if (endpoints)
    {
      if (!success && states.size() == 0)
        states.push_back(cloneState(s1));

      if (success)
        states.push_back(cloneState(s2));
    }

    return states.size();
  }

  robot_model::RobotModelConstPtr kmodel_;
private:

  robot_model_loader::RobotModelLoader robot_model_loader_;
};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_HYBRID_SPACE_INFORMATION_H
