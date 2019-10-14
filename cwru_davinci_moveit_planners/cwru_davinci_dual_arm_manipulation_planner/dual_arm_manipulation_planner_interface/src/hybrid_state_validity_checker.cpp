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
 * Description: The class to do state validity check
 */
#include <moveit/dual_arm_manipulation_planner_interface/hybrid_state_validity_checker.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/robot_state/conversions.h>


using namespace dual_arm_manipulation_planner_interface;
//using namespace davinci_moveit_object_handling;

HybridStateValidityChecker::HybridStateValidityChecker(const ros::NodeHandle &node_handle,
                                                       const std::string &robot_name,
                                                       const std::string &object_name,
                                                       const ompl::base::SpaceInformationPtr &si)
  : node_handle_(node_handle), robot_model_loader_(robot_name), robot_name_(robot_name),object_name_(object_name),
    ompl::base::StateValidityChecker(si)
{
  defaultSettings();

//  initializeIKPlugin();

  kmodel_ = robot_model_loader_.getModel();

  planning_scene_.reset(new planning_scene::PlanningScene(kmodel_));

//  pMonitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_name_));

  complete_initial_robot_state_.reset(new robot_state::RobotState(kmodel_));

//  tss_.reset(new TSStateStorage(*complete_initial_robot_state_));

  collision_request_with_distance_.distance = true;

  collision_request_with_cost_.cost = true;

  collision_request_simple_.contacts = true;
  collision_request_simple_.max_contacts = 1000;

  hyStateSpace_->validity_checking_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->validty_check_num = 0;

//  robot_state_publisher_ = node_handle_.advertise<moveit_msgs::DisplayRobotState>("collision_check_robot_state", 1);

  loadNeedleModel();
}

// std::chrono::duration<double> HybridStateValidityChecker::validity_checking_duration_ = std::chrono::duration<double>::zero();

bool HybridStateValidityChecker::isValid(const ompl::base::State *state) const
{
  auto start = std::chrono::high_resolution_clock::now();
  hyStateSpace_->validty_check_num += 1;

  const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(state);
  if (hs->isValidityKnown())
    return hs->isMarkedValid();

  bool is_valid = false;
  // check bounds
  if (!si_->satisfiesBounds(state))
    printf("Invalid State: Out of bound.");
  else
  {
    // convert ompl state to moveit robot state
//    robot_state::RobotState *kstate = tss_->getStateStorage();
    const robot_state::RobotStatePtr kstate(new robot_state::RobotState(kmodel_));
    if(!kstate)
      return is_valid;
    kstate->setToDefaultValues();
    const std::string selected_group_name = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";

    if (!convertObjectToRobotState(kstate, hs, selected_group_name))
      printf("Invalid State: No IK solution.");

    if(hs->jointsComputed())
    {
      moveit::core::AttachedBody* needle_model = createAttachedBody(selected_group_name, object_name_, hs->graspIndex().value);
      kstate->attachBody(needle_model);
      kstate->update();

      // TODO check feasibility
      planning_scene_->setCurrentState(*kstate);
      // check collision avoidance
      collision_detection::CollisionResult res;
      planning_scene_->checkCollision(collision_request_simple_, res, *kstate);
      if(res.collision)
      {
        ROS_INFO("Invalid State: Robot state is in collision with planning scene. \n");
        collision_detection::CollisionResult::ContactMap contactMap = res.contacts;
        for(collision_detection::CollisionResult::ContactMap::const_iterator it = contactMap.begin(); it != contactMap.end(); ++it)
        {
          ROS_INFO("Contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
        }
//        publishRobotState(*kstate);
      }

      kstate->clearAttachedBodies();
      is_valid = !res.collision;
      if(res.collision == true)
        const_cast<HybridObjectStateSpace::StateType*>(hs)->markInvalid();
      else
        const_cast<HybridObjectStateSpace::StateType*>(hs)->markValid();
    }
  }

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  hyStateSpace_->validity_checking_duration_ += elapsed;
  return is_valid;
}

double HybridStateValidityChecker::cost(const ompl::base::State* state) const
{
  double cost = 0.0;

//  robot_state::RobotState *kstate = tss_->getStateStorage();
  const robot_state::RobotStatePtr kstate(new robot_state::RobotState(kmodel_));
  if(!kstate)
    return false;
  kstate->setToDefaultValues();
  const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(state);
  const std::string selected_group_name = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";

  if(!convertObjectToRobotState(kstate, hs, selected_group_name))
  {
    printf("Invalid State: No IK solution.");
    return false;
  }

//  pMonitor_->requestPlanningSceneState();
//  planning_scene_monitor::LockedPlanningSceneRW ls(pMonitor_);
//  ls->setCurrentState(*kstate);
  planning_scene_->setCurrentState(*kstate);
  // Calculates cost from a summation of distance to obstacles times the size of the obstacle
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_with_cost_, res, *kstate);

  for (std::set<collision_detection::CostSource>::const_iterator it = res.cost_sources.begin() ; it != res.cost_sources.end() ; ++it)
    cost += it->cost * it->getVolume();

  return cost;
}


double HybridStateValidityChecker::clearance(const ompl::base::State* state) const
{
//  robot_state::RobotState *kstate = tss_->getStateStorage();
  const robot_state::RobotStatePtr kstate(new robot_state::RobotState(kmodel_));
  if(!kstate)
    return false;
  kstate->setToDefaultValues();
  const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(state);
  const std::string selected_group_name = (hs->armIndex().value == 1) ? "psm_one" : "psm_two";

  if(!convertObjectToRobotState(kstate, hs, selected_group_name))
  {
    printf("Invalid State: No IK solution.");
    return false;
  }
//  pMonitor_->requestPlanningSceneState();
//  planning_scene_monitor::LockedPlanningSceneRW ls(pMonitor_);
//  ls->setCurrentState(*kstate);
  planning_scene_->setCurrentState(*kstate);
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_with_distance_, res, *kstate);
  return res.collision ? 0.0 : (res.distance < 0.0 ? std::numeric_limits<double>::infinity() : res.distance);
}


bool HybridStateValidityChecker::convertObjectToRobotState(const moveit::core::RobotStatePtr &pRSstate,
                                                           const HybridObjectStateSpace::StateType *pHyState,
                                                           const std::string &selected_group_name) const
{
  if(!pHyState->jointsComputed())
  {
    // convert object pose to robot tip pose
    // this is the gripper tool tip link frame wrt /base_link
    Eigen::Affine3d object_pose;  // object pose w/rt base frame
//  si_->getStateSpace()->as<HybridObjectStateSpace>()->printState(hs, std::cout);
    hyStateSpace_->se3ToEigen3d(pHyState, object_pose);

    Eigen::Affine3d grasp_pose = hyStateSpace_->possible_grasps_[pHyState->graspIndex().value].grasp_pose;
    Eigen::Affine3d tool_tip_pose = object_pose * grasp_pose.inverse();

    const robot_state::JointModelGroup *selected_joint_model_group = pRSstate->getJointModelGroup(selected_group_name);
    std::size_t attempts = 2;
    double timeout = 1.0;
    bool found_ik = pRSstate->setFromIK(selected_joint_model_group, tool_tip_pose, attempts, timeout);

//  bool found_ik = setFromIK(rstate, selected_joint_model_group, selected_group_name,
//                            selected_joint_model_group->getOnlyOneEndEffectorTip()->getName(), tool_tip_pose);
    if(!found_ik)
    {
      const_cast<HybridObjectStateSpace::StateType *>(pHyState)->setJointsComputed(false);
      const_cast<HybridObjectStateSpace::StateType *>(pHyState)->markInvalid();
      return found_ik;
    }
    const_cast<HybridObjectStateSpace::StateType *>(pHyState)->setJointsComputed(true);
    pRSstate->copyJointGroupPositions(selected_joint_model_group, pHyState->jointVariables().values);
  }
  else
    pRSstate->setJointGroupPositions(selected_group_name, pHyState->jointVariables().values);

  const std::string rest_group_name = (selected_group_name == "psm_one") ? "psm_two" : "psm_one";
  const robot_state::JointModelGroup *rest_joint_model_group = pRSstate->getJointModelGroup(rest_group_name);
  pRSstate->setToDefaultValues(rest_joint_model_group, rest_group_name + "_home");
  std::string rest_group_eef_name = rest_joint_model_group->getAttachedEndEffectorNames()[0];
  const robot_state::JointModelGroup *rest_joint_model_group_eef = pRSstate->getJointModelGroup(rest_group_eef_name);
  pRSstate->setToDefaultValues(rest_joint_model_group_eef, rest_group_eef_name + "_home");

  return true;
}

moveit::core::AttachedBody*
HybridStateValidityChecker::createAttachedBody(const std::string &active_group,
                                               const std::string &object_name,
                                               const int grasp_pose_id) const
{
  const robot_state::JointModelGroup *arm_joint_group = kmodel_->getJointModelGroup(active_group);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  EigenSTL::vector_Affine3d attach_trans = {hyStateSpace_->possible_grasps_[grasp_pose_id].grasp_pose};

  const robot_state::JointModelGroup *eef_group = kmodel_->getJointModelGroup(
    arm_joint_group->getAttachedEndEffectorNames()[0]);

  std::vector<std::string> touch_links_list = eef_group->getLinkModelNames();
  std::set<std::string> touch_links(touch_links_list.begin(), touch_links_list.end());

  trajectory_msgs::JointTrajectory dettach_posture;
//  dettach_posture.header.stamp = ros::Time::now();
//  dettach_posture.header.frame_id = kmodel_->getModelFrame().c_str();
//  dettach_posture.joint_names = eef_group->getVariableNames();
//  dettach_posture.points.resize(3);
//  dettach_posture.points[0].positions.push_back(0.5);
//  dettach_posture.points[1].positions.push_back(0.5);
//  dettach_posture.points[2].positions.push_back(0.5);

  return new moveit::core::AttachedBody(tip_link,
                                        object_name,
                                        needleShapes_,
                                        attach_trans,
                                        touch_links,
                                        dettach_posture);
}

void HybridStateValidityChecker::defaultSettings()
{
  hyStateSpace_ = si_->getStateSpace().get()->as<HybridObjectStateSpace>();
  if (hyStateSpace_ == nullptr)
    throw ompl::Exception("No state space for motion validator");
}

void HybridStateValidityChecker::loadNeedleModel()
{
  Eigen::Vector3d scale_vec(0.025, 0.025, 0.025);
  shapes::Mesh *needle_mesh;
  shapes::ShapeMsg mesh_msg;
  try
  {
    needle_mesh = shapes::createMeshFromResource("package://cwru_davinci_geometry_models/"
                                                   "props/needle_r/mesh/needle_r4.dae",
                                                 scale_vec);
    if(!shapes::constructMsgFromShape(needle_mesh, mesh_msg))
      throw "Needle model is not loaded";
    const shapes::Shape* needle_shape = shapes::constructShapeFromMsg(mesh_msg);
    shapes::ShapeConstPtr needle_shape_ptr(needle_shape);
    needleShapes_ = {needle_shape_ptr};
  }
  catch(const char* exception)
  {
    std::cerr << "Error: " << exception << '\n';
  }
}

void HybridStateValidityChecker::initializeIKPlugin()
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

void HybridStateValidityChecker::publishRobotState(const robot_state::RobotState& rstate) const
{
  moveit_msgs::DisplayRobotState drstate;
  robot_state_publisher_.publish(drstate);
  ros::spinOnce();
  ros::Duration(1.0).sleep();

  moveit::core::robotStateToRobotStateMsg(rstate, drstate.state);

  robot_state_publisher_.publish(drstate);
  ros::spinOnce();
  ros::Duration(1.0).sleep();
}

bool HybridStateValidityChecker::setFromIK(robot_state::RobotState &rstate,
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
  double timeout = 2.0;
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
