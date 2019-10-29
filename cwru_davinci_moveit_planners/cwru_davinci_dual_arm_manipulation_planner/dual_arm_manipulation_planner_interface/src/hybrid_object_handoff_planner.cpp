/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Case Western Reserve University
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
 * Description: This class is to do object handoff planning with given initial state and
 *              goal state, the result is robot joint trajectories for davinci dual-arm manipulators
 */

#include <dual_arm_manipulation_planner_interface/hybrid_object_handoff_planner.h>

using namespace dual_arm_manipulation_planner_interface;
namespace ob = ompl::base;

HybridObjectHandoffPlanner::HybridObjectHandoffPlanner
  (
    const ob::State *start,
    const ob::State *Goal,
    const double se3BoundXAxisMin,
    const double se3BoundXAxisMax,
    const double se3BoundYAxisMin,
    const double se3BoundYAxisMax,
    const double se3BoundZAxisMin,
    const double se3BoundZAxisMax,
    const int armIdxLwBd,
    const int armIdxUpBd,
    const int graspIdxLwBd,
    const int graspIdxUpBd
  )
{
  m_pHyStateSpace = std::make_shared<HybridObjectStateSpace>(armIdxLwBd, armIdxLwBd, graspIdxLwBd, graspIdxUpBd);
  m_pHyStateSpace->setSE3Bounds([&]() -> ob::RealVectorBounds
                                  {
                                    ob::RealVectorBounds se3XYZBounds(3);
                                    se3XYZBounds.setLow(0, se3BoundXAxisMin);
                                    se3XYZBounds.setHigh(0, se3BoundXAxisMax);
                                    se3XYZBounds.setLow(1, se3BoundYAxisMin);
                                    se3XYZBounds.setHigh(1, se3BoundYAxisMax);
                                    se3XYZBounds.setLow(2, se3BoundZAxisMin);
                                    se3XYZBounds.setHigh(2, se3BoundZAxisMax);
                                    return se3XYZBounds;
                                  });


}
