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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Su Lu*/

#include "ompl/base/StateSpace.h"
#include "ompl/base/ScopedState.h"
#include "ompl/util/RandomNumbers.h"

#include <dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>

#include <gtest/gtest.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <limits>

using namespace dual_arm_manipulation_planner_interface;

namespace hybrid_planner_test
{

/** \brief Encapsulate basic tests for state spaces. This class
    should be used for every state space included with ompl, to
    ensure basic functionality works. */
class HybridObjectStateSpaceTester : public HybridObjectStateSpace
{
public:

  /** \brief Construct a testing setup for hybrid object state space \e
      space. When samples need to be taken to ensure certain
      functionality works, \e n samples are to be drawn. For
      distance comparisons, use an error margin of \e eps. */
  HybridObjectStateSpaceTester
  (
  int arm_idx_lw_bd,
  int arm_idx_up_bd,
  int grasp_idx_lw_bd,
  int grasp_idx_up_bd,
  const std::vector<cwru_davinci_grasp::GraspInfo>& possible_grasps,
  const std::shared_ptr<HybridObjectStateSpace>& pHyStateSpace,
  double eps = std::numeric_limits<double>::epsilon() * 10.0
  ) :
  HybridObjectStateSpace(arm_idx_lw_bd,
                         arm_idx_up_bd,
                         grasp_idx_lw_bd,
                         grasp_idx_up_bd,
                         possible_grasps),
  pHyStateSpace_(pHyStateSpace), eps_(eps)
  {

  }

  virtual ~HybridObjectStateSpaceTester()
  {
  }

  void testHandoff()
  {
    EXPECT_EQ(0, handOffsNum(1, 1, 1, 1, 1, 1));
    EXPECT_EQ(1, handOffsNum(1, 1, 1, 2, 2, 2));
    EXPECT_EQ(2, handOffsNum(1, 1, 1, 1, 2, 1));
    EXPECT_EQ(2, handOffsNum(1, 1, 1, 1, 2, 2));
    EXPECT_EQ(3, handOffsNum(1, 1, 1, 2, 1, 1));
    EXPECT_EQ(3, handOffsNum(1, 1, 1, 2, 2, 1));
  }

  /** \brief Test that interpolation works as expected and also test triangle inequality */
  void testInterpolation()
  {
    ompl::base::ScopedState<HybridObjectStateSpace> from(pHyStateSpace_);
    ompl::base::ScopedState<HybridObjectStateSpace> to(pHyStateSpace_);
    ompl::base::ScopedState<HybridObjectStateSpace> cstate(pHyStateSpace_);
  }

  /** \brief Test that states are correctly cloned*/
  void testCloneState()
  {
    ompl::base::ScopedState<> source(pHyStateSpace_);
    source.random();
    const ompl::base::State* clonedState = pHyStateSpace_->cloneState(source.get());
  }

  /** \brief Call all tests for the state space */
  void test()
  {
    testInterpolation();
    testCloneState();
  }

private:

  std::shared_ptr<HybridObjectStateSpace> pHyStateSpace_;
  ompl::RNG rng_;

  double eps_;
};
}
