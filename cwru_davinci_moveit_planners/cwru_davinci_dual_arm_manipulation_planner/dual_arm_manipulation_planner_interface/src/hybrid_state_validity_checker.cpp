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
#include <dual_arm_manipulation_planner_interface/hybrid_state_validity_checker.h>
#include <geometric_shapes/shape_operations.h>

#include <moveit/robot_state/conversions.h>

using namespace dual_arm_manipulation_planner_interface;

HybridStateValidityChecker::HybridStateValidityChecker
(
const ompl::base::SpaceInformationPtr& si,
const robot_model::RobotModelConstPtr& pRobotModel,
const std::string& objectName
)
: ompl::base::StateValidityChecker(si),
  kmodel_(pRobotModel),
  m_ObjectName(objectName)
{
  defaultSettings();

  planning_scene_.reset(new planning_scene::PlanningScene(kmodel_));

  collision_request_with_distance_.distance = true;

  collision_request_with_cost_.cost = true;

  collision_request_simple_.contacts = true;
  collision_request_simple_.max_contacts = 1000;

  hyStateSpace_->validity_checking_duration_ = std::chrono::duration<double>::zero();

  hyStateSpace_->validty_check_num = 0;

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("/world", moveit_visual_tools::DISPLAY_ROBOT_STATE_TOPIC, kmodel_));
  visual_tools_->loadRobotStatePub("/davinci");
  loadNeedleModel();
}

bool HybridStateValidityChecker::isValid(const ompl::base::State* state) const
{
  auto start = std::chrono::high_resolution_clock::now();
  hyStateSpace_->validty_check_num += 1;

  const auto *pHybridState = dynamic_cast<const HybridObjectStateSpace::StateType *>(state);
  if (!pHybridState)
  {
    return false;
  }

  if (pHybridState->isValidityKnown())
  {
    return pHybridState->isMarkedValid();
  }

  bool is_valid = false;
  if (!si_->satisfiesBounds(state))
  {
    printf("Invalid State: Out of bound. \n");
  }
  else
  {
    // convert ompl state to moveit robot state
    const robot_state::RobotStatePtr kstate = std::make_shared<robot_state::RobotState>(kmodel_);

    if (!hybridStateToRobotState(pHybridState, kstate))
    {
      return is_valid;
    }

    if (pHybridState->jointsComputed())
    {
      is_valid = (planning_scene_) ? (!planning_scene_->isStateColliding(*kstate)) : false;
      kstate->clearAttachedBodies();
      if (!is_valid)
      {
        const_cast<HybridObjectStateSpace::StateType*>(pHybridState)->markInvalid();
      }
      else
      {
        const_cast<HybridObjectStateSpace::StateType*>(pHybridState)->markValid();
      }
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

  const robot_state::RobotStatePtr kstate(new robot_state::RobotState(kmodel_));
  if (!kstate)
  {
    return false;
  }
  const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(state);

  if (!hybridStateToRobotState(hs, kstate))
  {
    printf("Invalid State: No IK solution. \n");
    return false;
  }

  planning_scene_->setCurrentState(*kstate);
  // Calculates cost from a summation of distance to obstacles times the size of the obstacle
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_with_cost_, res, *kstate);

  for (std::set<collision_detection::CostSource>::const_iterator it = res.cost_sources.begin() ; it != res.cost_sources.end() ; ++it)
  {
    cost += it->cost * it->getVolume();
  }

  return cost;
}


double HybridStateValidityChecker::clearance(const ompl::base::State* state) const
{
  const robot_state::RobotStatePtr kstate(new robot_state::RobotState(kmodel_));
  if (!kstate)
  {
    return false;
  }

  const auto *hs = static_cast<const HybridObjectStateSpace::StateType *>(state);

  if (!hybridStateToRobotState(hs, kstate))
  {
    printf("Invalid State: No IK solution. \n");
    return false;
  }

  planning_scene_->setCurrentState(*kstate);
  collision_detection::CollisionResult res;
  planning_scene_->checkCollision(collision_request_with_distance_, res, *kstate);
  return res.collision ? 0.0 : (res.distance < 0.0 ? std::numeric_limits<double>::infinity() : res.distance);
}


bool HybridStateValidityChecker::hybridStateToRobotState
(
const HybridObjectStateSpace::StateType* pHyState,
const robot_state::RobotStatePtr& pRSstate,
bool attachedObject
) const
{
  if (!pHyState || !pRSstate)
  {
    printf("HybridStateValidityChecker: Invalid state pointer. \n");
    return false;
  }

  pRSstate->setToDefaultValues();

  const std::string supportGroup = (pHyState->armIndex().value == 1) ? "psm_one" : "psm_two";
  // convert object pose to robot tip pose
  // this is the gripper tool tip link frame wrt /base_link
  Eigen::Affine3d object_pose;  // object pose w/rt base frame
  hyStateSpace_->se3ToEigen3d(pHyState, object_pose);

  if (!pHyState->jointsComputed())
  {
    Eigen::Affine3d grasp_pose = hyStateSpace_->graspTransformations()[pHyState->graspIndex().value].grasp_pose;
    Eigen::Affine3d tool_tip_pose = object_pose * grasp_pose.inverse();

    const robot_state::JointModelGroup *selected_joint_model_group = pRSstate->getJointModelGroup(supportGroup);
    std::size_t attempts = 1;
    double timeout = 0.005;
    bool found_ik = pRSstate->setFromIK(selected_joint_model_group, tool_tip_pose, attempts, timeout);

    if (!found_ik)
    {
      printf("HybridStateValidityChecker: No IK solution.\n");
      const_cast<HybridObjectStateSpace::StateType *>(pHyState)->setJointsComputed(false);
      const_cast<HybridObjectStateSpace::StateType *>(pHyState)->markInvalid();
      return found_ik;
    }
    const_cast<HybridObjectStateSpace::StateType *>(pHyState)->setJointsComputed(true);
    pRSstate->update();
    pRSstate->copyJointGroupPositions(selected_joint_model_group, pHyState->jointVariables().values);
  }
  else
  {
    pRSstate->update();
    pRSstate->setJointGroupPositions(supportGroup, pHyState->jointVariables().values);
  }

  setMimicJointPositions(pRSstate, supportGroup);

  if (attachedObject)
  {
    const Eigen::Affine3d tool_tip_pose = pRSstate->getGlobalLinkTransform(pRSstate->getJointModelGroup(supportGroup)->getOnlyOneEndEffectorTip());
    Eigen::Affine3d grasp_pose = tool_tip_pose.inverse() * object_pose;

    if (grasp_pose.isApprox(hyStateSpace_->graspTransformations()[pHyState->graspIndex().value].grasp_pose, 1e-4))
    {
      grasp_pose = hyStateSpace_->graspTransformations()[pHyState->graspIndex().value].grasp_pose;
    }
    // attach object to supporting joint group of robot
    moveit::core::AttachedBody *pNeedleModel = createAttachedBody(supportGroup,
                                                                  m_ObjectName,
                                                                  grasp_pose);
    pRSstate->attachBody(pNeedleModel);
    pRSstate->update();
  }

  // set joints state to resting joint group of robot
  const std::string restGroup = (supportGroup == "psm_one") ? "psm_two" : "psm_one";
  const robot_state::JointModelGroup *restJntModelGroup = pRSstate->getJointModelGroup(restGroup);
  pRSstate->setToDefaultValues(restJntModelGroup, restGroup + "_home");

  const std::string restGroupEef = restJntModelGroup->getAttachedEndEffectorNames()[0];
  const robot_state::JointModelGroup *restJntModelGroupEef = pRSstate->getJointModelGroup(restGroupEef);
  pRSstate->setToDefaultValues(restJntModelGroupEef, restGroupEef + "_home");
  pRSstate->update();

  return true;
}

bool HybridStateValidityChecker::hybridStateToRobotStateNoAttachedObject
(
const HybridObjectStateSpace::StateType* pHyState,
const robot_state::RobotStatePtr& pRSstate
) const
{
  return hybridStateToRobotState(pHyState, pRSstate, false);
}

moveit::core::AttachedBody*
HybridStateValidityChecker::createAttachedBody
(
const std::string& supportGroup,
const std::string& objectName,
const int grasp_pose_id
) const
{
  const robot_state::JointModelGroup *arm_joint_group = kmodel_->getJointModelGroup(supportGroup);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  EigenSTL::vector_Affine3d attach_trans = {hyStateSpace_->graspTransformations()[grasp_pose_id].grasp_pose};

  const robot_state::JointModelGroup *eef_group = kmodel_->getJointModelGroup(
    arm_joint_group->getAttachedEndEffectorNames()[0]);

  std::vector<std::string> touch_links_list = eef_group->getLinkModelNames();
  std::set<std::string> touch_links(touch_links_list.begin(), touch_links_list.end());

  trajectory_msgs::JointTrajectory dettach_posture;
  return new moveit::core::AttachedBody(tip_link,
                                        objectName,
                                        needleShapes_,
                                        attach_trans,
                                        touch_links,
                                        dettach_posture);
}

moveit::core::AttachedBody*
HybridStateValidityChecker::createAttachedBody
(
const std::string& supportGroup,
const std::string& objectName,
const Eigen::Affine3d& grasp_pose
) const
{
  const robot_state::JointModelGroup *arm_joint_group = kmodel_->getJointModelGroup(supportGroup);
  const moveit::core::LinkModel *tip_link = arm_joint_group->getOnlyOneEndEffectorTip();

  EigenSTL::vector_Affine3d attach_trans = {grasp_pose};

  const robot_state::JointModelGroup *eef_group = kmodel_->getJointModelGroup(
    arm_joint_group->getAttachedEndEffectorNames()[0]);

  std::vector<std::string> touch_links_list = eef_group->getLinkModelNames();
  std::set<std::string> touch_links(touch_links_list.begin(), touch_links_list.end());

  trajectory_msgs::JointTrajectory dettach_posture;
  return new moveit::core::AttachedBody(tip_link,
                                        objectName,
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
  Eigen::Vector3d scale_vec(1.0, 1.0, 1.0);
  shapes::Mesh *needle_mesh;
  shapes::ShapeMsg mesh_msg;
  try
  {
    needle_mesh = shapes::createMeshFromResource("package://sim_gazebo/"
                                                 "props/needle/mesh/needle_thin.dae",
                                                 scale_vec);
    if (!shapes::constructMsgFromShape(needle_mesh, mesh_msg))
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

void HybridStateValidityChecker::publishRobotState(const robot_state::RobotState& rstate) const
{
  visual_tools_->publishRobotState(rstate);
  ros::Duration(1.0).sleep();
}

void HybridStateValidityChecker::setMimicJointPositions
(
const robot_state::RobotStatePtr& rstate,
const std::string& planning_group
) const
{
  const std::string outer_pitch_joint = (planning_group == "psm_one") ? "PSM1_outer_pitch" : "PSM2_outer_pitch";
  const double *joint_val = rstate->getJointPositions(outer_pitch_joint);
  if (joint_val)
    rstate->setJointGroupPositions(planning_group + "_base_mimics", joint_val);
}

bool HybridStateValidityChecker::noCollision
(
const robot_state::RobotState& rstate,
const std::string& planningGroup,
bool needleInteraction,
bool verbose
) const
{
  auto start_ik = std::chrono::high_resolution_clock::now();

  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_detection::CollisionResult collision_result;

  if (!needleInteraction)
  {
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
    const std::string psm = (planningGroup == "psm_one") ? "PSM1" : "PSM2";
    acm.setEntry(m_ObjectName, psm + "_tool_wrist_sca_ee_link_1", true);
    acm.setEntry(m_ObjectName, psm + "_tool_wrist_sca_ee_link_2", true);
    planning_scene_->checkCollision(collision_request, collision_result, rstate, acm);
  }
  else
  {
    planning_scene_->checkCollision(collision_request, collision_result, rstate);
  }

  if (collision_result.collision && verbose)
  {
    ROS_WARN("NoCollision: Robot state is in collision with planning scene. \n");
    collision_detection::CollisionResult::ContactMap contactMap = collision_result.contacts;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = contactMap.begin();
         it != contactMap.end(); ++it)
    {
      ROS_WARN("Contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
    }
  }

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->collision_checking_duration_ += elapsed;

  return !collision_result.collision;
}

void HybridStateValidityChecker::noCollisionThread
(
uint8_t& noCollision,
const robot_state::RobotState& rstate
) const
{
  std::lock_guard<std::mutex> guard(planning_scene_mutex_);

  auto start_ik = std::chrono::high_resolution_clock::now();

  planning_scene_->setCurrentState(rstate);
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_detection::CollisionResult collision_result;
  planning_scene_->checkCollision(collision_request, collision_result, rstate);
  noCollision = (!collision_result.collision) ? 1 : 0;

  auto finish_ik = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish_ik - start_ik;
  hyStateSpace_->collision_checking_duration_ += elapsed;

  if (collision_result.collision)
  {
    ROS_INFO("Invalid State: Robot state is in collision with planning scene. \n");
    collision_detection::CollisionResult::ContactMap contactMap = collision_result.contacts;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = contactMap.begin();
         it != contactMap.end(); ++it)
    {
      ROS_INFO("Contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
    }
  }
}

bool HybridStateValidityChecker::isRobotStateValid
(
const planning_scene::PlanningScene& planning_scene,
const std::string& planning_group,
bool needleInteraction,
bool verbose,
robot_state::RobotState* state,
const robot_state::JointModelGroup* group,
const double* ik_solution
) const
{
  state->setJointGroupPositions(group, ik_solution);
  const std::string outer_pitch_joint = (planning_group == "psm_one") ? "PSM1_outer_pitch" : "PSM2_outer_pitch";
  const double *joint_val = state->getJointPositions(outer_pitch_joint);
  if (joint_val)
    state->setJointGroupPositions(planning_group + "_base_mimics", joint_val);
  state->update();

  if (&planning_scene == nullptr)
  {
    return false;
  }

  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_detection::CollisionResult collision_result;

  if (!needleInteraction)
  {
    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
    const std::string psm = (planning_group == "psm_one") ? "PSM1" : "PSM2";
    acm.setEntry(m_ObjectName, psm + "_tool_wrist_sca_ee_link_1", true);
    acm.setEntry(m_ObjectName, psm + "_tool_wrist_sca_ee_link_2", true);
    planning_scene.checkCollision(collision_request, collision_result, *state, acm);
  }
  else
  {
    planning_scene.checkCollision(collision_request, collision_result, *state);
  }

  if (collision_result.collision && verbose)
  {
    ROS_WARN("IsRobotStateValid: Robot state is in collision with planning scene. \n");
    collision_detection::CollisionResult::ContactMap contactMap = collision_result.contacts;
    for (collision_detection::CollisionResult::ContactMap::const_iterator it = contactMap.begin();
         it != contactMap.end(); ++it)
    {
      ROS_WARN("Contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
    }
  }

  return !collision_result.collision;
}

bool HybridStateValidityChecker::validateTrajectory
(
const std::string& planningGroup,
const robot_state::RobotStatePtr& pRobotState,
const std::vector<std::vector<double>>& armJntTraj,
double jaw,
bool verbose,
bool needleInteraction
)
{
  setJawPosition(jaw, planningGroup, pRobotState);
  for (std::size_t i = 0; i < armJntTraj.size(); ++i)
  {
    pRobotState->setJointGroupPositions(planningGroup, armJntTraj[i]);
    setMimicJointPositions(pRobotState, planningGroup);
    pRobotState->update();
    if (!noCollision(*pRobotState, "", needleInteraction, verbose))
    {
      return false;
    }
  }

  return true;
}

bool HybridStateValidityChecker::detechNeedleGrasped
(
const robot_state::RobotState& rstate,
const std::string& planningGroup,
bool verbose
)
{
  collision_detection::CollisionRequest collision_request;
  collision_request.contacts = true;
  collision_detection::CollisionResult collision_result;
  planning_scene_->checkCollision(collision_request, collision_result, rstate);

  bool isNeedleGrasp = false;
  if (!collision_result.collision)  // if no collision found, then needle is not grasped
    return isNeedleGrasp;

  // found collision, try to turn off needle interaction to see if collision still present
  // turning off the needle interaction
  collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrix();
  const std::string psm = (planningGroup == "psm_one") ? "PSM1" : "PSM2";
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_WARN("DetechNeedleGrasped found contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
  }
  acm.setEntry(m_ObjectName, psm + "_tool_wrist_sca_ee_link_1", true);
  acm.setEntry(m_ObjectName, psm + "_tool_wrist_sca_ee_link_2", true);

  // after turning off needle interaction, do collision again
  collision_result.clear();
  planning_scene_->checkCollision(collision_request, collision_result, rstate, acm);

  if (collision_result.collision)  // after turn off needleInteraction still found collision
  {
    if (verbose)
    {
      ROS_WARN("DetechNeedleGrasped: Robot state is in collision with planning scene. \n");
      collision_detection::CollisionResult::ContactMap contactMap = collision_result.contacts;
      for (collision_detection::CollisionResult::ContactMap::const_iterator it = contactMap.begin(); it != contactMap.end(); ++it)
      {
        ROS_WARN("Contact between: %s and %s \n", it->first.first.c_str(), it->first.second.c_str());
      }
    }

    // this is failure due to collision detected with other link
    return isNeedleGrasp;
  }

  // after turn off needleInteraction no collision found,
  // this means collision only happens btw needle and gripper links
  isNeedleGrasp = true;
  return isNeedleGrasp;
}

void HybridStateValidityChecker::setJawPosition
(
double radian,
const std::string& planningGroup,
const robot_state::RobotStatePtr& pRState
)
{
  const std::string eef_group_name = pRState->getJointModelGroup(planningGroup)->getAttachedEndEffectorNames()[0];
  std::vector<double> eef_joint_position;
  pRState->copyJointGroupPositions(eef_group_name, eef_joint_position);

  for (std::size_t i = 0; i < eef_joint_position.size(); ++i)
  {
    eef_joint_position[i] = radian;
  }

  pRState->setJointGroupPositions(eef_group_name, eef_joint_position);
  setMimicJointPositions(pRState, planningGroup);  // this line cannot be removed, as copyJointGroupPosition does not copy mimic joints's value
  pRState->update();
}