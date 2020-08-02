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
 * Description: The main function to do needle handoff calculation then to control robot to
 *              execute trajectories
 */

#include <dual_arm_manipulation_planner_interface/davinci_needle_handoff_execution_manager.h>

using namespace dual_arm_manipulation_planner_interface;
namespace ob = ompl::base;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "davinci_needle_handoff_execution_main");

  ros::NodeHandle nodeHandle;
  ros::NodeHandle nodeHandlePriv("~");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration(3.0).sleep();

  if (!nodeHandlePriv.hasParam("object_name"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `object_name` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << nodeHandlePriv.getNamespace());
    return false;
  }
  std::string objectName;
  nodeHandlePriv.getParam("object_name", objectName);

  if (!nodeHandlePriv.hasParam("initial_support_arm"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `initial_support_arm` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << nodeHandlePriv.getNamespace());
    return false;
  }
  std::string initialSupportArm;
  nodeHandlePriv.getParam("initial_support_arm", initialSupportArm);

  if (!nodeHandlePriv.hasParam("planning_time"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `planning_time` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << nodeHandlePriv.getNamespace());
    return false;
  }
  double planningTime;
  nodeHandlePriv.getParam("planning_time", planningTime);

  if (!nodeHandlePriv.hasParam("goal_state"))
  {
    ROS_ERROR_STREAM("Handoff planning inputs parameter `goal_state` missing "
                     "from rosparam server. "
                     "Searching in namespace: "
                     << nodeHandlePriv.getNamespace());
    return false;
  }

  double goalStateAry[15];
  XmlRpc::XmlRpcValue xmlGoalStateArray;
  nodeHandlePriv.getParam("goal_state", xmlGoalStateArray);
  if (xmlGoalStateArray.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (std::size_t i = 0; i < xmlGoalStateArray.size(); ++i)
    {
      ROS_ASSERT(xmlGoalStateArray[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      goalStateAry[i] = static_cast<double>(xmlGoalStateArray[i]);
    }
  }
  else
  {
    ROS_ERROR_STREAM("SE3 bounds type is not type array?");
  }

  ob::ScopedState<ob::SE3StateSpace> goalNeedlePose(std::make_shared<ob::SE3StateSpace>());
  goalNeedlePose->setXYZ(goalStateAry[0],
                         goalStateAry[1],
                         goalStateAry[2]);

  goalNeedlePose->rotation().x = goalStateAry[3];
  goalNeedlePose->rotation().y = goalStateAry[4];
  goalNeedlePose->rotation().z = goalStateAry[5];
  goalNeedlePose->rotation().w = goalStateAry[6];

  const std::vector<double> goalJointPosition = {goalStateAry[7], goalStateAry[8], goalStateAry[9],
                                                 goalStateAry[10], goalStateAry[11], goalStateAry[12]};

  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr pSimpleGrasp =
  boost::make_shared<cwru_davinci_grasp::DavinciSimpleNeedleGrasper>(nodeHandle,
                                                                     nodeHandlePriv,
                                                                     initialSupportArm,
                                                                     objectName);

  // Try defined grasped fist
  if (pSimpleGrasp->pickNeedle(objectName, cwru_davinci_grasp::NeedlePickMode::DEFINED))
  {
    ROS_INFO("%s: needle picked up in DEFINED way", nodeHandle.getNamespace().c_str());
    return 0;
  }

  if (!pSimpleGrasp->pickNeedle(objectName, cwru_davinci_grasp::NeedlePickMode::FINDGOOD))
  {
    ROS_INFO("%s: did not pick needle up in FINDGOOD way", nodeHandle.getNamespace().c_str());
    return -1;
  }

  ros::Duration(10.0).sleep();
  ros::spinOnce();
  // execute needle grasping first
  std::vector<cwru_davinci_grasp::GraspInfo> graspPoses = pSimpleGrasp->getAllPossibleNeedleGrasps(false);
  cwru_davinci_grasp::GraspInfo initialGraspInfo = pSimpleGrasp->getSelectedGraspInfo();

  geometry_msgs::Pose needlePose = pSimpleGrasp->getNeedlePose().pose;
  ob::ScopedState<ob::SE3StateSpace> startNeedlePose(std::make_shared<ob::SE3StateSpace>());
  initialGraspInfo = graspPoses[initialGraspInfo.graspParamInfo.grasp_id];
  startNeedlePose->setXYZ(needlePose.position.x,
                          needlePose.position.y,
                          needlePose.position.z);
  startNeedlePose->rotation().x = needlePose.orientation.x;
  startNeedlePose->rotation().y = needlePose.orientation.y;
  startNeedlePose->rotation().z = needlePose.orientation.z;
  startNeedlePose->rotation().w = needlePose.orientation.w;

  DavinciNeedleHandoffExecutionManager needleHandoffExecutor(nodeHandle,
                                                             nodeHandlePriv,
                                                             graspPoses,
                                                             objectName);

  needleHandoffExecutor.constructStartAndGoalState(startNeedlePose.get(),
                                                   (initialSupportArm == "psm_one" ? 1 : 2),
                                                   initialGraspInfo.graspParamInfo.grasp_id,
                                                   pSimpleGrasp->graspedJointPosition(),
                                                   goalNeedlePose.get(),
                                                   (int) goalStateAry[13],
                                                   (int) goalStateAry[14],
                                                   goalJointPosition);

  if (!needleHandoffExecutor.initializePlanner())
  {
    return -1;
  }

  if (!needleHandoffExecutor.planNeedleHandoffTraj(planningTime))
  {
    return -1;
  }

  if (!needleHandoffExecutor.executeNeedleHandoffTraj())
  {
    return -1;
  }

  ros::shutdown();
  return 0;
}
