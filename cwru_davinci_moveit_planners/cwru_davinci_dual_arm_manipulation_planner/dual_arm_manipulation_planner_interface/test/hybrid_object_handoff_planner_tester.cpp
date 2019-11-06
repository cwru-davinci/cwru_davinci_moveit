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
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>

#include <dual_arm_manipulation_planner_interface/hybrid_object_handoff_planner.h>
#include <gtest/gtest.h>

using namespace dual_arm_manipulation_planner_interface;
namespace ob = ompl::base;

namespace hybrid_planner_test
{
class HybridObjectHandoffPlannerTester : public HybridObjectHandoffPlanner
{
public:
  HybridObjectHandoffPlannerTester
  (
  const robot_model::RobotModelPtr& pRobotModel
  );

  virtual ~HybridObjectHandoffPlannerTester() {}

  void initializePlanner
  (
  const robot_model::RobotModelConstPtr& pRobotModel
  );

  void testConnectStates();
};

HybridObjectHandoffPlannerTester::HybridObjectHandoffPlannerTester
(
const robot_model::RobotModelPtr& pRobotModel
)
 : HybridObjectHandoffPlanner()
{
  initializePlanner(pRobotModel);
}

void HybridObjectHandoffPlannerTester::initializePlanner
(
const robot_model::RobotModelConstPtr& pRobotModel
)
{
  ASSERT_FALSE(!m_pHyStateSpace);
  setupSpaceInformation(m_pHyStateSpace, pRobotModel,"needle_r");
  EXPECT_TRUE(m_pSpaceInfor.get());
}

void HybridObjectHandoffPlannerTester::testConnectStates()
{
  ob::PlannerDataStorage dataStorage;
  ob::PlannerData data(m_pSpaceInfor);

  std::string dataPath = ros::package::getPath("cwru_davinci_dual_arm_manipulation_planner");
  dataStorage.load((dataPath + "/../../../" + "HybridRRTPlannerData").c_str(), data);

}
}

