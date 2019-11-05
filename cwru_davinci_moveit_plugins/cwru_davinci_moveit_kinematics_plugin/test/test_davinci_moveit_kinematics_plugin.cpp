/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Su Lu */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <pluginlib/class_loader.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <eigen_conversions/eigen_msg.h>
// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/rdf_loader/rdf_loader.h>

#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <urdf_interface/model.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <cwru_davinci_kinematics/davinci_kinematic_definitions.h>
#include <cwru_davinci_moveit_kinematics_plugin/davinci_moveit_kinematics_constants.h>

#include <fstream>
#include <iostream>
#include <cstdlib> // for exit()

#define IK_NEAR_ORIENTATION 1e-5
#define IK_NEAR_TRANSLATE 1e-5

bool isWithinJointLimit(const std::vector<double> &joint_angles)
{
  //  davinci_kinematics::davinci_kinematic_definitions
  bool is_within = true;
  double q_min;
  double q_max;
  double q_upper_limits_copy[] = {1.0, 0.7, 0.23, 2.25, 1.57, 1.39, 1.57};

  double q_lower_limits_copy[] = {-1.0, -0.7, 0.016, -2.25, -1.57, -1.39, -1.57};

  for(int i = 0; i < joint_angles.size(); i++)
  {
    q_min = q_lower_limits_copy[i];
    q_max = q_upper_limits_copy[i];
    if(joint_angles[i] < q_min || joint_angles[i] > q_max)
    {
      is_within = false;
      return is_within;
    }
  }
  return is_within;
}

double findMin(const double &a, const double &b)
{
  std::vector<double> list = {fabs(a - b), fabs(a - b - 2 * M_PI), fabs(a - b + 2 * M_PI)};
  double min = list[0];
  for(int i = 1; i < list.size(); i++)
  {
    if(min > list[i])
    {
      min = list[i];
    }
  }
  return min;
}

bool isJointSetEqual(std::vector<double> &joint_set_1, std::vector<double> &joint_set_2)
{
  bool is_equal = false;

  if(joint_set_1.size() != joint_set_2.size())
  {
    return is_equal;
  }

  Eigen::VectorXd diff_vector(joint_set_1.size());

  for(int i = 0; i < joint_set_1.size(); i++)
  {
    diff_vector[i] = findMin(joint_set_1[i], joint_set_2[i]);
  }

  if(diff_vector.norm() <= IK_NEAR_ORIENTATION)
  {
    is_equal = true;
  }

  return is_equal;
}

void isEqual(const double &a, const double &b, const double &tol, int &count)
{
  if(abs(a - b) < tol)
  {
    count += 1;
  }
}

bool isTwoVectorEqual(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
  Eigen::Vector3d diff_vec = a - b;
  //  diff_vec = diff_vec.normalized();
  double diff = diff_vec.norm();
  if(diff <= IK_NEAR_TRANSLATE)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool isTwoRotationEqual(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
{
  Eigen::Matrix3d diff = a * b.inverse() - Eigen::Matrix3d::Identity();

  //  ROS_INFO_STREAM(diff);

  if(diff.norm() < IK_NEAR_ORIENTATION)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool isTwoPoseEqual(const Eigen::Affine3d &pose_1, const Eigen::Affine3d &pose_2)
{
  Eigen::Vector3d pose_1_vec = pose_1.translation();
  Eigen::Vector3d pose_2_vec = pose_2.translation();

  Eigen::Matrix3d rot_1 = pose_1.linear();
  Eigen::Matrix3d rot_2 = pose_2.linear();

  if(isTwoVectorEqual(pose_1_vec, pose_2_vec) && isTwoRotationEqual(rot_1, rot_2))
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool isTwoPoseEqual(const geometry_msgs::Pose &pose_1, const geometry_msgs::Pose &pose_2)
{
  Eigen::Affine3d affine_pose_1;
  Eigen::Affine3d affine_pose_2;

  tf::poseMsgToEigen(pose_1, affine_pose_1);
  tf::poseMsgToEigen(pose_2, affine_pose_2);

  return isTwoPoseEqual(affine_pose_1, affine_pose_2);
}

class DavinciMoveitKinematicsPluginTest
{
public:

  boost::shared_ptr<kinematics::KinematicsBase> m_pKinematicsSolver;
  boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> m_pKinematicsLoader;

  bool initialize()
  {
    double search_discretization;
    ros::NodeHandle nh("~");
    m_pKinematicsSolver = NULL;
    m_pKinematicsLoader.reset(
      new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
    std::string plugin_name;

    if(!nh.getParam("plugin_name", plugin_name))
    {
      ROS_ERROR("No plugin name found on parameter server");
      EXPECT_TRUE(0);
      return false;
    }
    ROS_INFO("Plugin name: %s", plugin_name.c_str());
    try
    {
      m_pKinematicsSolver = m_pKinematicsLoader->createInstance(plugin_name);
    }
    catch(pluginlib::PluginlibException &ex)//handle the class failing to load
    {
      ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
      EXPECT_TRUE(0);
      return false;
    }

    std::string root_name, tip_name;
    ros::WallTime start_time = ros::WallTime::now();
    bool done = true;
    while((ros::WallTime::now() - start_time).toSec() <= 5.0)
    {
      if(!nh.getParam("root_name", root_name))
      {
        ROS_ERROR("No root name found on parameter server");
        done = false;
        EXPECT_TRUE(0);
        continue;
      }
      if(!nh.getParam("tip_name", tip_name))
      {
        ROS_ERROR("No tip name found on parameter server");
        done = false;
        EXPECT_TRUE(0);
        continue;
      }
      if(!nh.getParam("search_discretization", search_discretization))
      {
        ROS_ERROR("No search discretization found on parameter server");
        done = false;
        EXPECT_TRUE(0);
        continue;
      }
      done = true;
    }

    if(!done)
    {
      return false;
    }

    if(m_pKinematicsSolver->initialize("robot_description", "psm_one", root_name, tip_name, search_discretization))
    {
      return true;
    }
    else
    {
      EXPECT_TRUE(0);
      return false;
    }
  };

  void pose_callback(const geometry_msgs::Pose &ik_pose,
                     const std::vector<double> &joint_state,
                     moveit_msgs::MoveItErrorCodes &error_code)
  {
    error_code.val = error_code.SUCCESS;
  }

  void joint_state_callback(const geometry_msgs::Pose &ik_pose,
                            const std::vector<double> &joint_state,
                            moveit_msgs::MoveItErrorCodes &error_code)
  {
    std::vector<std::string> link_names;
    link_names.push_back("PSM1_psm_base_link");
    std::vector<geometry_msgs::Pose> solutions;
    solutions.resize(1);

    if(!m_pKinematicsSolver->getPositionFK(link_names, joint_state, solutions))
    {
      error_code.val = error_code.PLANNING_FAILED;
    }
    if(solutions[0].position.z > 0.0)
    {
      error_code.val = error_code.SUCCESS;
    }
    else
    {
      error_code.val = error_code.PLANNING_FAILED;
    }
  }

  //  kinematics::KinematicsBase* m_pKinematicsSolver;

};

DavinciMoveitKinematicsPluginTest ik_plugin_test;

TEST(ArmIKPlugin, DISABLED_TestMimicJoints)
{
  rdf_loader::RDFLoader rdf_loader;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface> &urdf_model = rdf_loader.getURDF();
  const robot_model::RobotModelPtr pRModelUrdfSrdf(new robot_model::RobotModel(urdf_model, srdf));
  const robot_state::RobotStatePtr pRStateUrdfSrdf(new robot_state::RobotState(pRModelUrdfSrdf));

  robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
  const robot_model::RobotModelConstPtr pKModelRobotDes = robotModelLoader.getModel();
  const robot_state::RobotStatePtr pRState(new robot_state::RobotState(pKModelRobotDes));

  int psm_base_active_joint_num = 6;
  int psm_base_mimic_joint_num = 5;
  int psm_base_all_joint_num = 11;
  const robot_state::JointModelGroup* psm_one_base_with_mimic = pRStateUrdfSrdf->getJointModelGroup("psm_one");
  std::vector<std::string> psm_one_base_all_joints_name = psm_one_base_with_mimic->getJointModelNames();
//  EXPECT_EQ(psm_base_all_joint_num, psm_one_base_all_joints_name.size());

//  EXPECT_EQ(psm_one_base_all_joints_name[0], "PSM1_outer_yaw");
//  EXPECT_EQ(psm_one_base_all_joints_name[1], "PSM1_outer_pitch");
//  EXPECT_EQ(psm_one_base_all_joints_name[2], "PSM1_outer_insertion");
//  EXPECT_EQ(psm_one_base_all_joints_name[3], "PSM1_outer_roll");
//  EXPECT_EQ(psm_one_base_all_joints_name[4], "PSM1_outer_wrist_pitch");
//  EXPECT_EQ(psm_one_base_all_joints_name[5], "PSM1_outer_wrist_yaw");
//  EXPECT_EQ(psm_one_base_all_joints_name[6], "PSM1_outer_pitch_1");
//  EXPECT_EQ(psm_one_base_all_joints_name[7], "PSM1_outer_pitch_3");
//  EXPECT_EQ(psm_one_base_all_joints_name[8], "PSM1_outer_pitch_5");
//  EXPECT_EQ(psm_one_base_all_joints_name[9], "PSM1_outer_pitch_4");
//  EXPECT_EQ(psm_one_base_all_joints_name[10],"PSM1_outer_pitch_2");

  const robot_state::JointModelGroup* psm_one_base_no_mimic = pRState->getJointModelGroup("psm_one");
  std::vector<std::string> psm_one_base_active_joints_name= psm_one_base_no_mimic->getJointModelNames();
  EXPECT_EQ(psm_base_active_joint_num, psm_one_base_active_joints_name.size());

  EXPECT_EQ(psm_one_base_active_joints_name[0], "PSM1_outer_yaw");
  EXPECT_EQ(psm_one_base_active_joints_name[1], "PSM1_outer_pitch");
  EXPECT_EQ(psm_one_base_active_joints_name[2], "PSM1_outer_insertion");
  EXPECT_EQ(psm_one_base_active_joints_name[3], "PSM1_outer_roll");
  EXPECT_EQ(psm_one_base_active_joints_name[4], "PSM1_outer_wrist_pitch");
  EXPECT_EQ(psm_one_base_active_joints_name[5], "PSM1_outer_wrist_yaw");
}

TEST(ArmIKPlugin, TestKDLKinematics)
{
  robot_model_loader::RobotModelLoader robotModelLoader("robot_description");
  const robot_model::RobotModelConstPtr pKModel = robotModelLoader.getModel();
  ROS_INFO("Model frame: %s", pKModel->getModelFrame().c_str());
  ros::NodeHandle nh("~");
  double time_out = 0;
  int test_num = 0;
  nh.getParam("time_out", time_out);
  nh.getParam("test_num", test_num);

  int succeeded_num = 0;

  // fk robot state
  const robot_state::RobotStatePtr pRStateFK(new robot_state::RobotState(pKModel));
  pRStateFK->setToDefaultValues();
  const robot_state::JointModelGroup* arm_joint_group_fk = pRStateFK->getJointModelGroup("psm_one");
  const std::vector<std::string> psmOneActiveJointNames = arm_joint_group_fk->getVariableNames();
  EXPECT_EQ(6, psmOneActiveJointNames.size());

  EXPECT_EQ(psmOneActiveJointNames[0], "PSM1_outer_yaw");
  EXPECT_EQ(psmOneActiveJointNames[1], "PSM1_outer_pitch");
  EXPECT_EQ(psmOneActiveJointNames[2], "PSM1_outer_insertion");
  EXPECT_EQ(psmOneActiveJointNames[3], "PSM1_outer_roll");
  EXPECT_EQ(psmOneActiveJointNames[4], "PSM1_outer_wrist_pitch");
  EXPECT_EQ(psmOneActiveJointNames[5], "PSM1_outer_wrist_yaw");

  const moveit::core::LinkModel *tip_link = arm_joint_group_fk->getOnlyOneEndEffectorTip();
  std::vector<double> randomPositionByFK;

  // ik robot state
  const robot_state::RobotStatePtr pRStateIK(new robot_state::RobotState(pKModel));
  pRStateIK->setToDefaultValues();
  const robot_state::JointModelGroup* arm_joint_group_ik = pRStateIK->getJointModelGroup("psm_one");
  std::vector<double> positionByIK;
  int active_joint_num = 6;
  int all_joints_num = 28;
  ros::WallTime start_time = ros::WallTime::now();
  for(size_t i = 0; i < test_num; i++)
  {
    pRStateFK->setToRandomPositions(arm_joint_group_fk);
    pRStateFK->update();
    pRStateFK->copyJointGroupPositions("psm_one", randomPositionByFK);
    EXPECT_EQ(active_joint_num, randomPositionByFK.size());
    EXPECT_EQ(all_joints_num, pRStateFK->getVariableCount());
    const double * fk_array = pRStateFK->getVariablePositions();
    std::vector<double> jointVariableByFK(fk_array, fk_array + all_joints_num);
    EXPECT_EQ(all_joints_num, jointVariableByFK.size());
    for(size_t ii = 0; ii < jointVariableByFK.size(); ++ii)
    {
      EXPECT_NEAR(jointVariableByFK[ii], pRStateFK->getVariablePosition(pRStateFK->getVariableNames()[ii]), 1e-11);
      if(!((jointVariableByFK[ii] - pRStateFK->getVariablePosition(pRStateFK->getVariableNames()[ii])) < 1e-11))
      {
        ROS_INFO("\n FK error joint angle in joint %s \n", pRStateFK->getVariableNames()[ii].c_str());
      }
    }
    const Eigen::Affine3d goal_tool_tip_pose = pRStateFK->getGlobalLinkTransform(tip_link);

    bool found_ik = pRStateIK->setFromIK(arm_joint_group_ik, goal_tool_tip_pose, 1, time_out);
    pRStateIK->update();
    if(found_ik)
    {
      succeeded_num += 1;
      pRStateFK->copyJointGroupPositions("psm_one", positionByIK);
      EXPECT_EQ(randomPositionByFK.size(), positionByIK.size());
      EXPECT_EQ(active_joint_num, positionByIK.size());
      EXPECT_EQ(all_joints_num, pRStateIK->getVariableCount());

      const double * ik_array = pRStateIK->getVariablePositions();
      std::vector<double> jointVariableByIK(ik_array, ik_array + all_joints_num);
      EXPECT_EQ(all_joints_num, jointVariableByIK.size());
      for(size_t ii = 0; ii < jointVariableByIK.size(); ++ii)
      {
        EXPECT_NEAR(jointVariableByIK[ii], pRStateIK->getVariablePosition(pRStateIK->getVariableNames()[ii]), 1e-11);
        if(!((jointVariableByIK[ii] - pRStateIK->getVariablePosition(pRStateIK->getVariableNames()[ii])) < 1e-11))
        {
          ROS_INFO("\n IK error joint angle in joint %s \n", pRStateIK->getVariableNames()[ii].c_str());
        }
      }

      for(size_t jj = 0; jj < jointVariableByIK.size(); ++jj)
      {
        EXPECT_NEAR(jointVariableByIK[jj], jointVariableByFK[jj], 1e-3);
        ROS_INFO("\n IK and FK joint: %s, %f \n", pRStateIK->getVariableNames()[jj].c_str(), jointVariableByFK[jj]);
        if(!(fabs(jointVariableByIK[jj] - jointVariableByFK[jj]) < 1e-3))
        {
          ROS_INFO("\n IK and FK error joint angle in joint %s \n", pRStateIK->getVariableNames()[jj].c_str());
        }
      }

      for(size_t j = 0; j < randomPositionByFK.size(); j++)
      {
        EXPECT_NEAR(randomPositionByFK[j], positionByIK[j], 1e-5);
      }
    }
  }
  ROS_INFO("Success Rate: %f", (double) succeeded_num / test_num);
  bool success_count = (succeeded_num >= 0.9999 * test_num) ? true : false;
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now() - start_time).toSec());
}

//TEST(ArmIKPlugin, TestInitialize)
TEST(ArmIKPlugin, DISABLED_TestInitialize)
{
  ASSERT_TRUE(ik_plugin_test.initialize());
  //   Test getting chain information
  std::string root_name = ik_plugin_test.m_pKinematicsSolver->getBaseFrame();
  EXPECT_TRUE(root_name == std::string("world"));

  std::string tip_name = ik_plugin_test.m_pKinematicsSolver->getTipFrame();
  EXPECT_TRUE(tip_name == std::string("PSM1_tool_tip_link"));

  std::vector<std::string> joint_names = ik_plugin_test.m_pKinematicsSolver->getJointNames();
  EXPECT_EQ((int) joint_names.size(), davinci_moveit_kinematics::NUM_JOINTS_ARM7DOF);

  EXPECT_EQ(joint_names[0], "PSM1_outer_yaw");
  EXPECT_EQ(joint_names[1], "PSM1_outer_pitch");
  EXPECT_EQ(joint_names[2], "PSM1_outer_insertion");
  EXPECT_EQ(joint_names[3], "PSM1_outer_roll");
  EXPECT_EQ(joint_names[4], "PSM1_outer_wrist_pitch");
  EXPECT_EQ(joint_names[5], "PSM1_outer_wrist_yaw");
}

//TEST(ArmIKPlugin, TestFK)
TEST(ArmIKPlugin, DISABLED_TestFK)
{
  rdf_loader::RDFLoader rdf_loader;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface> &urdf_model = rdf_loader.getURDF();
  const robot_model::RobotModelPtr pKinematicModel(new robot_model::RobotModel(urdf_model, srdf));

  //  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //  robot_model::RobotModelPtr pKinematicModel = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", pKinematicModel->getModelFrame().c_str());
  robot_state::JointModelGroup *joint_model_group = pKinematicModel->getJointModelGroup(
    ik_plugin_test.m_pKinematicsSolver->getGroupName());

  //Test FK
  std::vector<double> seed, fk_values, solution;

  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(ik_plugin_test.m_pKinematicsSolver->getTipFrame());
  const robot_state::RobotStatePtr pKinematicState(new robot_state::RobotState(pKinematicModel));
  pKinematicState->setToDefaultValues();


  ros::NodeHandle nh("~");
  int test_num = 1000;
  int success = 0;
  ros::WallTime start_time = ros::WallTime::now();

  for(int i = 0; i < test_num; ++i)
  {
    seed.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);
    fk_values.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

    pKinematicState->setToRandomPositions(joint_model_group);  // populate psm one joints with random values
    pKinematicState->update();
    pKinematicState->copyJointGroupPositions(joint_model_group, fk_values);  // get random set joints values

    for(int i = 0; i < fk_values.size(); ++i)
    {
      std::cout << fk_values[i] << ' ';
    }
    std::cout << std::endl;

    std::vector<geometry_msgs::Pose> poses;
    poses.resize(1);
    bool result_fk = ik_plugin_test.m_pKinematicsSolver->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);

    Eigen::Affine3d fk_tip_wrt_world;
    tf::poseMsgToEigen(poses[0], fk_tip_wrt_world);

    Eigen::Affine3d ik_tip_wrt_world = pKinematicState->getGlobalLinkTransform(
      ik_plugin_test.m_pKinematicsSolver->getTipFrame());

    bool result = isTwoPoseEqual(ik_tip_wrt_world, fk_tip_wrt_world);
    EXPECT_TRUE(result);
    if(result)
    {
      success += 1;
    }
  }

  double correct_rate = (double)(success / test_num);
  ROS_INFO("Success Rate: %f", correct_rate);
  bool success_count = (success > 0.9999 * test_num) ? true : false;
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now() - start_time).toSec());
}


//TEST(ArmIKPlugin, TestCompareFK)
TEST(ArmIKPlugin, DISABLED_TestCompareFK)
{
  rdf_loader::RDFLoader rdf_loader;
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface> &urdf_model = rdf_loader.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));

  //  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  //  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(
    ik_plugin_test.m_pKinematicsSolver->getGroupName());

  //Test FK
  std::vector<double> seed, fk_values, solution;

  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(ik_plugin_test.m_pKinematicsSolver->getTipFrame());
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  std::vector<geometry_msgs::Pose> poses;

  ros::NodeHandle nh("~");
  int number_fk_compare_tests = 100;
  int success = 0;
  nh.setParam("number_fk_compare_tests", number_fk_compare_tests);
  ros::WallTime start_time = ros::WallTime::now();


  for(int i = 0; i < number_fk_compare_tests; ++i)
  {
    seed.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);
    fk_values.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

    kinematic_state->setToRandomPositions(joint_model_group);  // populate psm one joints with random values
    kinematic_state->copyJointGroupPositions(joint_model_group, fk_values);

    poses.resize(1);
    bool result_fk = ik_plugin_test.m_pKinematicsSolver->getPositionFK(fk_names, fk_values, poses);
    ASSERT_TRUE(result_fk);
    Eigen::Affine3d fk_by_kdl;
    tf::poseMsgToEigen(poses[0], fk_by_kdl);

    // FK computed by davinci
    davinci_kinematics::Vectorq7x1 q_vec;

    for(int i = 0; i < fk_values.size(); i++)
    {
      q_vec(i) = fk_values[i];
    }

    davinci_kinematics::Forward davinci_fk;

    Eigen::Affine3d affine_fk_pose_wrt_base = davinci_fk.fwd_kin_solve(q_vec);

    Eigen::Affine3d affine_base_wrt_world = kinematic_state->getFrameTransform("PSM1_psm_base_link");

    Eigen::Affine3d affine_fk_pose_wrt_world = affine_base_wrt_world * affine_fk_pose_wrt_base;

    //    tf::poseEigenToMsg(affine_fk_pose_wrt_world, fk_pose_by_davinci[0]);

    // compare fk results
    bool result = isTwoPoseEqual(affine_fk_pose_wrt_world, fk_by_kdl);
    EXPECT_TRUE(result);
    if(result)
    {
      success += 1;
    }
  }

  ROS_INFO("Success Rate: %f", (double) success / number_fk_compare_tests);
  bool success_count = (success > 0.99 * number_fk_compare_tests);
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now() - start_time).toSec());
}


//TEST(ArmIKPlugin, TestSearchIK)
TEST(ArmIKPlugin, DISABLED_TestSearchIK)
{
  //  ASSERT_TRUE(ik_plugin_test.initialize());
  rdf_loader::RDFLoader rdf_loader;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface> &urdf_model = rdf_loader.getURDF();
  const robot_model::RobotModelPtr pKinematicModel(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup *joint_model_group = pKinematicModel->getJointModelGroup(
    ik_plugin_test.m_pKinematicsSolver->getGroupName());

  //Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 2.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(ik_plugin_test.m_pKinematicsSolver->getTipFrame());

  const robot_state::RobotStatePtr pKinematicState(new robot_state::RobotState(pKinematicModel));
  pKinematicState->setToDefaultValues();

  ros::NodeHandle nh("~");
  int number_ik_tests = 3000;
  int acctual_test_num = 0;
  unsigned int ik_found_success = 0;
  unsigned int ik_correct_success = 0;
  unsigned int fk_found_success = 0;

  const char *path = "/home/sulu/failed_joint_angle_sets.txt";
  std::ofstream output(path);
  if(!output)
  {
    // Print an error and exit
    ROS_ERROR("file could not be opened for writing!");
    std::exit(1);
  }

  std::vector<geometry_msgs::Pose> failed_ik_pose;
  failed_ik_pose.clear();

  std::vector< std::vector<double> > failed_ik_joint_sets;
  failed_ik_joint_sets.clear();

  ros::WallTime start_time = ros::WallTime::now();
  for(unsigned int i = 0; i < (unsigned int) number_ik_tests; ++i)
  {
    Eigen::Affine3d affine_base_wrt_world = pKinematicState->getFrameTransform("PSM1_psm_base_link");

    seed.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);
    fk_values.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

    pKinematicState->setToRandomPositions(joint_model_group);
    pKinematicState->copyJointGroupPositions(joint_model_group, fk_values);

    acctual_test_num++;

    std::vector<geometry_msgs::Pose> poses_wrt_world;
    poses_wrt_world.resize(1);
    bool result_fk = ik_plugin_test.m_pKinematicsSolver->getPositionFK(fk_names, fk_values, poses_wrt_world);
    if(result_fk)
    {
      fk_found_success++;
    }

    // transform pose wrt world to pose wrt base frame
    Eigen::Affine3d affine_poses_wrt_world;
    tf::poseMsgToEigen(poses_wrt_world[0], affine_poses_wrt_world);

    Eigen::Affine3d affine_poses_wrt_base;
    affine_poses_wrt_base = affine_base_wrt_world.inverse() * affine_poses_wrt_world;

    std::vector<geometry_msgs::Pose> poses_wrt_base;
    poses_wrt_base.resize(1);
    tf::poseEigenToMsg(affine_poses_wrt_base, poses_wrt_base[0]);

    bool result_ik = ik_plugin_test.m_pKinematicsSolver->searchPositionIK(poses_wrt_base[0], seed, timeout, solution,
                                                                        error_code);
    if(result_ik)
    {
      ik_found_success++;
      result_ik = ik_plugin_test.m_pKinematicsSolver->getPositionIK(poses_wrt_base[0], seed, solution, error_code);
      if(result_ik)
      {
        std::vector<geometry_msgs::Pose> new_poses_wrt_world;
//        new_poses_wrt_world.resize(1);
//        result_fk = ik_plugin_test.m_pKinematicsSolver->getPositionFK(fk_names, solution, new_poses_wrt_world);

        if(isJointSetEqual(fk_values, solution))
        {
          ik_correct_success++;
        }
      }
    }
    else
    {
      std::vector<geometry_msgs::Pose> new_poses_wrt_world;
      new_poses_wrt_world.resize(1);
      result_fk = ik_plugin_test.m_pKinematicsSolver->getPositionFK(fk_names, solution, new_poses_wrt_world);
      if(result_fk)
      {
        failed_ik_pose.push_back(new_poses_wrt_world[0]);
        failed_ik_joint_sets.push_back(fk_values);
      }
      ROS_INFO("The random joints angle results in IK failure: \n");
      davinci_kinematics::Forward davinci_fk;
      davinci_kinematics::Vectorq7x1 q_vec;
      // copy failed to find ik solution joint list
      for(int i = 0; i < fk_values.size(); i++)
      {
        q_vec(i) = fk_values[i];
      }

      Eigen::Affine3d affine_fk_pose_wrt_base = davinci_fk.fwd_kin_solve(q_vec);

      //      Eigen::Affine3d affine_base_wrt_world = pKinematicState->getFrameTransform("PSM1_psm_base_link");

      //      Eigen::Affine3d affine_fk_pose_wrt_world = affine_base_wrt_world * affine_fk_pose_wrt_base;

      //      geometry_msgs::Pose fk_pose_wrt_base;
      //
      //      tf::poseEigenToMsg(affine_fk_pose_wrt_base, fk_pose_wrt_base);

      davinci_kinematics::Inverse davinci_ik;

      int ik_solve_return_val = davinci_ik.ik_solve(affine_fk_pose_wrt_base);

      switch(ik_solve_return_val)
      {
        case -6 :
        {
          //          ROS_INFO("davinci ik solver has no solution");
          for(int i = 0; i < fk_values.size(); i++)
          {
            output << fk_values[i] << "\n";
          }
          output << "\n";
          break;
        }
        case 1 :
        {
          ROS_INFO("davinci ik solver has one solution");
          std::vector<double> solution_ik;
          davinci_kinematics::Vectorq7x1 vectorq = davinci_ik.get_soln();
          solution_ik.resize(davinci_moveit_kinematics::NUM_JOINTS_ARM7DOF, 0.0);
          for(int i = 0; i < solution_ik.size(); i++)
          {
            solution_ik[i] = vectorq(i);
          }
          //          davinci_moveit_kinematics::convertVectorq7x1ToStdVector(davinci_ik.get_soln(), solution_ik);
          //          solution.push_back(solution_ik);
          ik_found_success++;
          if(isJointSetEqual(fk_values, solution_ik))
          {
            ik_correct_success++;
          }
          break;
        }

        default:
        {
          ROS_INFO("davinci ik solver has multiple solutions which is not allowed");
          break;
        }
      }
    }
  }
  output << "There are " << (acctual_test_num - ik_found_success)
         << " sets of joint values failed to have IK solution."
         << "\n";
  output.close();
  ROS_INFO_STREAM(
    "There are " << (number_ik_tests - acctual_test_num) << " sets of joint values out of joint limits." << "\n");
  ROS_INFO("IK Found Success Rate: %f", (double) ik_found_success / acctual_test_num);
  ROS_INFO("IK Found and Correct Success Rate: %f", (double) ik_correct_success / ik_found_success);
  bool success_count = (ik_found_success > 0.99 * acctual_test_num);
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now() - start_time).toSec());

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(ik_plugin_test.m_pKinematicsSolver->getGroupName());
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = false;
  for(int i = 0; i < failed_ik_joint_sets.size(); i++)
  {
    pKinematicState->setToDefaultValues();
    move_group.setJointValueTarget(failed_ik_joint_sets[i]);
    success = (bool) move_group.plan(my_plan);
    ROS_INFO_NAMED("IK search test", "(joint space goal) %s", success ? "" : "FAILED");
    ros::Duration(1.0).sleep();
  }


//  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_for_failed_ik_pose", 0 );
//  visualization_msgs::Marker marker;
//  marker.header.frame_id = "/world";
//  marker.header.stamp = ros::Time();
//  marker.ns = "my_namespace";
//  marker.id = 0;
//  marker.type = visualization_msgs::Marker::SPHERE;
//  marker.action = visualization_msgs::Marker::ADD;
////  marker.pose.position.x = 0;
////  marker.pose.position.y = 0;
////  marker.pose.position.z = 0;
//  marker.pose.orientation.x = 0.0;
//  marker.pose.orientation.y = 0.0;
//  marker.pose.orientation.z = 0.0;
//  marker.pose.orientation.w = 1.0;
//  marker.scale.x = 0.01;
//  marker.scale.y = 0.01;
//  marker.scale.z = 0.01;
//  marker.color.a = 1.0;
//  marker.color.r = 1.0;
//  marker.color.g = 0.0;
//  marker.color.b = 0.0;

//  for(int i = 0; i < 100; i++)
//  {
//    marker.header.stamp = ros::Time();
//    marker.points.clear(); // clear out this vector
//    for(int i = 0; i < failed_ik_pose.size(); i++)
//    {
//      geometry_msgs::Point point;
//      point.x = failed_ik_pose[i].position.x;
//      point.y = failed_ik_pose[i].position.y;
//      point.z = failed_ik_pose[i].position.z;
//      marker.points.push_back(point);
//    }

//    ros::Duration(0.5).sleep();
//    vis_pub.publish( marker );
//    ros::spinOnce();
//  }

}

TEST(ArmIKPlugin, DISABLED_moveAlongWorldXTest)
//TEST(ArmIKPlugin, moveAlongWorldXTest)
{
  rdf_loader::RDFLoader rdf_loader;
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface> &urdf_model = rdf_loader.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(
    ik_plugin_test.m_pKinematicsSolver->getGroupName());

  //Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 2.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(ik_plugin_test.m_pKinematicsSolver->getTipFrame());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  Eigen::Affine3d affine_tool_tip_pose_wrt_world = kinematic_state->getFrameTransform("PSM1_tool_tip_link");
  double init_tip_offset_wrt_world = fabs(affine_tool_tip_pose_wrt_world.matrix()(0, 3));
  double move_x_dist = 0.15;
  double final_tip_offset_wrt_world = init_tip_offset_wrt_world - move_x_dist;
  double traveled_dist = 0.0;
  double resolution = 0.01;

  ros::NodeHandle nh("~");
  int number_ik_tests = 0;
  int failed_tool_tip_sets_num = 0;
  //  int acctual_test_num = 0;
  //  nh.param("number_ik_tests", number_ik_tests, 3000);
  unsigned int ik_found_success = 0;
  //  unsigned int ik_correct_success = 0;
  //  unsigned int fk_found_success = 0;

  const char *path1 = "/home/sulu/failed_tool_tip_pose.txt";
  std::ofstream output(path1);
  if(!output)
  {
    // Print an error and exit
    std::exit(1);
    ROS_ERROR("file could not be opened for writing!");
  }

  const char *path2 = "/home/sulu/joint_angle_sets.txt";
  std::ofstream output2(path2);
  if(!output2)
  {
    // Print an error and exit
    std::exit(1);
    ROS_ERROR("file could not be opened for writing!");
  }

  ros::WallTime start_time = ros::WallTime::now();
  while((init_tip_offset_wrt_world - traveled_dist) > final_tip_offset_wrt_world)
  {
//    number_ik_tests++;
//    affine_tool_tip_pose_wrt_world.linear() << 0, 0, 1,
//                                              -1, 0, 0,
//                                               0, -1, 0;

//    affine_tool_tip_pose_wrt_world.matrix()(2, 3) = 0.30;

//    affine_tool_tip_pose_wrt_world.matrix()(0, 3) = -(init_tip_offset_wrt_world - (number_ik_tests *
//                                                                                   resolution));  // subtraction means tool tip is going to insert



    Eigen::Affine3d affine_base_wrt_world = kinematic_state->getFrameTransform("PSM1_psm_base_link");
    ROS_INFO_STREAM(affine_base_wrt_world.matrix());
    Eigen::Affine3d affine_tool_tip_goal_wrt_base = affine_base_wrt_world.inverse() * affine_tool_tip_pose_wrt_world;

    std::cout << affine_tool_tip_goal_wrt_base.matrix() << "\n";
    std::vector<geometry_msgs::Pose> poses_wrt_base;
    poses_wrt_base.resize(1);
    tf::poseEigenToMsg(affine_tool_tip_goal_wrt_base, poses_wrt_base[0]);

    seed.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);
    fk_values.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

    bool result_ik = ik_plugin_test.m_pKinematicsSolver->searchPositionIK(poses_wrt_base[0], seed, timeout, solution,
                                                                        error_code);
    if(result_ik)
    {
      ik_found_success++;
      result_ik = ik_plugin_test.m_pKinematicsSolver->getPositionIK(poses_wrt_base[0], seed, solution, error_code);
      if(result_ik)
      {
        output2 << "ik solution joint set:" << "\n";
        for(int i = 0; i < solution.size(); i++)
        {
          output2 << solution[i] << "\n";
        }
      }
      output2 << "\n";
    }
    else
    {
      davinci_kinematics::Inverse davinci_ik;

      int ik_solve_return_val = davinci_ik.ik_solve(affine_tool_tip_goal_wrt_base);

      switch(ik_solve_return_val)
      {
        case -6 :
        {
          failed_tool_tip_sets_num++;
          ROS_INFO("davinci ik solver still has no solution");
          output << "tool tip pose goal set: " << failed_tool_tip_sets_num << "\n";
          output << "rotation part:" << "\n" << affine_tool_tip_goal_wrt_base.linear() << "\n";
          output << "\n";
          output << "translation part:" << "\n" << affine_tool_tip_goal_wrt_base.translation() << "\n";
          output << "\n";

          kinematic_state->copyJointGroupPositions(joint_model_group, fk_values);
          output2 << "The joint set which has no corresponding ik solution:" << "\n";
          for(int i = 0; i < fk_values.size(); i++)
          {
            output2 << fk_values[i] << "\n";
          }
          output2 << "\n";

          break;
        }
        case 1 :
        {
          ROS_INFO("davinci ik solver has one solution");
          //          std::vector<double> solution_ik;
          //          convertVectorq7x1ToStdVector(davinci_inverse_.get_soln(), solution_ik);
          //          solution.push_back(solution_ik);
          ik_found_success++;
          //          correct_success++;
          break;
        }

        default:
        {
          ROS_INFO("davinci ik solver has multiple solutions which is not allowed");
          break;
        }
      }
    }
    traveled_dist += resolution;
  }
  output << "There are " << (number_ik_tests - ik_found_success)
         << " sets of tool tip pose failed to have IK solution."
         << "\n";
  output.close();
  output2.close();
  ROS_INFO("IK Found Success Rate: %f", (double) ik_found_success / number_ik_tests);
  bool success_count = (ik_found_success > 0.99 * number_ik_tests);
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now() - start_time).toSec());
}

TEST(ArmIKPlugin, DISABLED_specificJointSetIkTest)
//TEST(ArmIKPlugin, specificJointSetIkTest)
{
  rdf_loader::RDFLoader rdf_loader;
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface> &urdf_model = rdf_loader.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(
    ik_plugin_test.m_pKinematicsSolver->getGroupName());

  //Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 2.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(ik_plugin_test.m_pKinematicsSolver->getTipFrame());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  std::vector<std::vector<double> > joint_sets;
  joint_sets.clear();
  fk_values.clear();
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.015);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  joint_sets.push_back(fk_values);

  fk_values.clear();
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.014);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  joint_sets.push_back(fk_values);

  fk_values.clear();
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.013);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  joint_sets.push_back(fk_values);

  fk_values.clear();
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.012);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  joint_sets.push_back(fk_values);

  fk_values.clear();
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.011);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  fk_values.push_back(0.0);
  joint_sets.push_back(fk_values);

  ros::NodeHandle nh("~");
  int number_ik_tests = joint_sets.size();
//  int failed_tool_tip_sets_num = 0;
  //  int acctual_test_num = 0;
  //  nh.param("number_ik_tests", number_ik_tests, 3000);
  unsigned int ik_found_success = 0;
  unsigned int ik_correct_success = 0;
  //  unsigned int fk_found_success = 0;

  ros::WallTime start_time = ros::WallTime::now();
  for(int i = 0; i < joint_sets.size(); i++)
  {
    Eigen::Affine3d affine_base_wrt_world = kinematic_state->getFrameTransform("PSM1_psm_base_link");

    seed.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

    //    affine_base_wrt_world= kinematic_state->getFrameTransform("PSM1_psm_base_link");
    std::vector<geometry_msgs::Pose> poses_wrt_world;
    poses_wrt_world.resize(1);
    bool result_fk = ik_plugin_test.m_pKinematicsSolver->getPositionFK(fk_names, joint_sets[i], poses_wrt_world);
    ASSERT_TRUE(result_fk);

    // transform pose wrt world to pose wrt base frame
    Eigen::Affine3d affine_poses_wrt_world;
    tf::poseMsgToEigen(poses_wrt_world[0], affine_poses_wrt_world);

    Eigen::Affine3d affine_poses_wrt_base;
    affine_poses_wrt_base = affine_base_wrt_world.inverse() * affine_poses_wrt_world;

    std::vector<geometry_msgs::Pose> poses_wrt_base;
    poses_wrt_base.resize(1);
    tf::poseEigenToMsg(affine_poses_wrt_base, poses_wrt_base[0]);

    bool result_ik = ik_plugin_test.m_pKinematicsSolver->searchPositionIK(poses_wrt_base[0], seed, timeout, solution,
                                                                        error_code);
    if(result_ik)
    {
      ik_found_success++;
      result_ik = ik_plugin_test.m_pKinematicsSolver->getPositionIK(poses_wrt_base[0], seed, solution, error_code);
      if(result_ik)
      {
        if(isJointSetEqual(fk_values, solution))
        {
          ik_correct_success++;
        }
      }

    }
  }
//  output << "There are " << (acctual_test_num - ik_found_success)
//         << " sets of joint values failed to have IK solution."
//         << "\n";
//  output.close();
//  ROS_INFO_STREAM(
//    "There are " << (number_ik_tests - acctual_test_num) << " sets of joint values out of joint limits." << "\n");
  ROS_INFO("IK Found Success Rate: %f", (double) ik_found_success / number_ik_tests);
  ROS_INFO("IK Found and Correct Success Rate: %f", (double) ik_correct_success / ik_found_success);
  bool success_count = (ik_found_success > 0.99 * number_ik_tests);
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now() - start_time).toSec());
}

TEST(ArmIKPlugin, DISABLED_verticalInsertionTest)
//TEST(ArmIKPlugin, verticalInsertionTest)
{
  //  ASSERT_TRUE(ik_plugin_test.initialize());
  rdf_loader::RDFLoader rdf_loader;
  robot_model::RobotModelPtr kinematic_model;
  const boost::shared_ptr<srdf::Model> &srdf = rdf_loader.getSRDF();
  const boost::shared_ptr<urdf::ModelInterface> &urdf_model = rdf_loader.getURDF();
  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(
    ik_plugin_test.m_pKinematicsSolver->getGroupName());

  //Test inverse kinematics
  std::vector<double> seed, fk_values, solution;
  double timeout = 2.0;
  moveit_msgs::MoveItErrorCodes error_code;
  solution.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

  std::vector<std::string> fk_names;
  fk_names.push_back(ik_plugin_test.m_pKinematicsSolver->getTipFrame());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  Eigen::Affine3d affine_tool_tip_pose_wrt_world = kinematic_state->getFrameTransform("PSM1_tool_tip_link");
  double init_tip_height_wrt_world = affine_tool_tip_pose_wrt_world.matrix()(2, 3);
  double insertion_dist = 0.15;
  double final_tip_height_wrt_world = init_tip_height_wrt_world - insertion_dist;
  double traveled_dist = 0.0;
  double resolution = 0.001;

  ros::NodeHandle nh("~");
  int number_ik_tests = 0;
  int failed_tool_tip_sets_num = 0;
  //  int acctual_test_num = 0;
  //  nh.param("number_ik_tests", number_ik_tests, 3000);
  unsigned int ik_found_success = 0;
  //  unsigned int ik_correct_success = 0;
  //  unsigned int fk_found_success = 0;

  const char *path1 = "/home/sulu/failed_tool_tip_pose.txt";
  std::ofstream output(path1);
  if(!output)
  {
    // Print an error and exit
    std::exit(1);
    ROS_ERROR("file could not be opened for writing!");
  }

  const char *path2 = "/home/sulu/joint_angle_sets.txt";
  std::ofstream output2(path2);
  if(!output2)
  {
    // Print an error and exit
    std::exit(1);
    ROS_ERROR("file could not be opened for writing!");
  }

  ros::WallTime start_time = ros::WallTime::now();
  while((init_tip_height_wrt_world - traveled_dist) > final_tip_height_wrt_world)
  {
    number_ik_tests++;
    affine_tool_tip_pose_wrt_world.matrix()(2, 3) =
      init_tip_height_wrt_world - (number_ik_tests * resolution);  // subtraction means tool tip is going to insert

    Eigen::Affine3d affine_base_wrt_world = kinematic_state->getFrameTransform("PSM1_psm_base_link");
    Eigen::Affine3d affine_tool_tip_goal_wrt_base = affine_base_wrt_world.inverse() * affine_tool_tip_pose_wrt_world;

    std::vector<geometry_msgs::Pose> poses_wrt_base;
    poses_wrt_base.resize(1);
    tf::poseEigenToMsg(affine_tool_tip_goal_wrt_base, poses_wrt_base[0]);

    seed.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);
    fk_values.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

    bool result_ik = ik_plugin_test.m_pKinematicsSolver->searchPositionIK(poses_wrt_base[0], seed, timeout, solution,
                                                                        error_code);
    if(result_ik)
    {
      ik_found_success++;
      result_ik = ik_plugin_test.m_pKinematicsSolver->getPositionIK(poses_wrt_base[0], seed, solution, error_code);
      output2 << "ik solution joint set:" << "\n";
      for(int i = 0; i < solution.size(); i++)
      {
        output2 << solution[i] << "\n";
      }
      output2 << "\n";
    }
    else
    {
      davinci_kinematics::Inverse davinci_ik;

      int ik_solve_return_val = davinci_ik.ik_solve(affine_tool_tip_goal_wrt_base);

      switch(ik_solve_return_val)
      {
        case -6 :
        {
          failed_tool_tip_sets_num++;
          ROS_INFO("davinci ik solver still has no solution");
          output << "tool tip pose goal set: " << failed_tool_tip_sets_num << "\n";
          output << "rotation part:" << "\n" << affine_tool_tip_goal_wrt_base.linear() << "\n";
          output << "\n";
          output << "translation part:" << "\n" << affine_tool_tip_goal_wrt_base.translation() << "\n";
          output << "\n";

          kinematic_state->copyJointGroupPositions(joint_model_group, fk_values);
          output2 << "The joint set which has no corresponding ik solution:" << "\n";
          for(int i = 0; i < fk_values.size(); i++)
          {
            output2 << fk_values[i] << "\n";
          }
          output2 << "\n";

          break;
        }
        case 1 :
        {
          ROS_INFO("davinci ik solver has one solution");
          //          std::vector<double> solution_ik;
          //          convertVectorq7x1ToStdVector(davinci_inverse_.get_soln(), solution_ik);
          //          solution.push_back(solution_ik);
          ik_found_success++;
          //          correct_success++;
          break;
        }

        default:
        {
          ROS_INFO("davinci ik solver has multiple solutions which is not allowed");
          break;
        }
      }
    }
    traveled_dist += resolution;
  }
  output << "There are " << (number_ik_tests - ik_found_success)
         << " sets of tool tip pose failed to have IK solution."
         << "\n";
  output.close();

//  output2 << "There are " << (number_ik_tests - ik_found_success)
//         << " sets of tool tip pose failed to have IK solution."
//         << "\n";
  output2.close();
  ROS_INFO("IK Found Success Rate: %f", (double) ik_found_success / number_ik_tests);
  bool success_count = (ik_found_success > 0.99 * number_ik_tests);
  EXPECT_TRUE(success_count);
  ROS_INFO("Elapsed time: %f", (ros::WallTime::now() - start_time).toSec());
}

//TEST(ArmIKPlugin, searchIK)
//{
////  ASSERT_TRUE(ik_plugin_test.initialize());
//  rdf_loader::RDFLoader rdf_loader;
//  robot_model::RobotModelPtr kinematic_model;
//  const boost::shared_ptr<srdf::Model>& srdf = rdf_loader.getSRDF();
//  const boost::shared_ptr<urdf::ModelInterface>& urdf_model = rdf_loader.getURDF();
//  kinematic_model.reset(new robot_model::RobotModel(urdf_model, srdf));

////  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
////  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());


//  robot_model::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup(
//    ik_plugin_test.m_pKinematicsSolver->getGroupName());

//  //Test inverse kinematics
//  std::vector<double> seed, fk_values, solution;
//  double timeout = 2.0;
//  moveit_msgs::MoveItErrorCodes error_code;
//  solution.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

//  std::vector<std::string> fk_names;
//  fk_names.push_back(ik_plugin_test.m_pKinematicsSolver->getTipFrame());

//  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
//  kinematic_state->setToDefaultValues();

//  Eigen::Affine3d affine_base_wrt_world= kinematic_state->getFrameTransform("PSM1_psm_base_link");

//  ros::NodeHandle nh("~");
//  int number_ik_tests;
//  nh.param("number_ik_tests", number_ik_tests, 100);
//  unsigned int found_success = 0;
//  unsigned int correct_success = 0;

//  std::ofstream output;
//  output.open("failed joint angle sets.txt");
//  if (!output)
//  {
////    Print an error and exit
//    ROS_ERROR("file could not be opened for writing!");
//    std::exit(1);
//  }

//  ros::WallTime start_time = ros::WallTime::now();
//  for(unsigned int i=0; i < (unsigned int) number_ik_tests; ++i)
//  {
//    Eigen::Affine3d affine_base_wrt_world= kinematic_state->getFrameTransform("PSM1_psm_base_link");
////    ROS_INFO("In IK test, the number of joints in total is %d", (int) ik_plugin_test.kinematics_solver->getJointNames().size());
////    ROS_INFO_STREAM("Base wrt World Translation: \n" << affine_base_wrt_world.translation());
////    ROS_INFO_STREAM("Base wrt World Rotation: \n" << affine_base_wrt_world.rotation());


//    seed.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);
//    fk_values.resize(ik_plugin_test.m_pKinematicsSolver->getJointNames().size(), 0.0);

//    kinematic_state->setToRandomPositions(joint_model_group);
//    kinematic_state->copyJointGroupPositions(joint_model_group, fk_values);

//    affine_base_wrt_world= kinematic_state->getFrameTransform("PSM1_psm_base_link");
//    std::vector<geometry_msgs::Pose> poses_wrt_world;
//    poses_wrt_world.resize(1);
//    bool result_fk = ik_plugin_test.m_pKinematicsSolver->getPositionFK(fk_names, fk_values, poses_wrt_world);
//    ASSERT_TRUE(result_fk);

//    std::vector<Eigen::Affine3d> affine_poses_wrt_world;
//    affine_poses_wrt_world.resize(1);
//    tf::poseMsgToEigen(poses_wrt_world[0], affine_poses_wrt_world[0]);
////    ROS_INFO_STREAM("Poses wrt World Translation: \n" << affine_poses_wrt_world[0].translation());
////    ROS_INFO_STREAM("Poses wrt World Rotation: \n" << affine_poses_wrt_world[0].rotation());

//    std::vector<Eigen::Affine3d> affine_poses_wrt_base;
//    affine_poses_wrt_base.resize(1);
//    affine_poses_wrt_base[0] = affine_base_wrt_world.inverse() * affine_poses_wrt_world[0];

////    ROS_INFO_STREAM("Poses wrt Base Translation: \n" << affine_poses_wrt_base[0].translation());
////    ROS_INFO_STREAM("Poses wrt Base Rotation: \n" << affine_poses_wrt_base[0].rotation());

//    std::vector<geometry_msgs::Pose> poses_wrt_base;
//    poses_wrt_base.resize(1);
//    tf::poseEigenToMsg(affine_poses_wrt_base[0], poses_wrt_base[0]);

//    bool found_result = ik_plugin_test.m_pKinematicsSolver->searchPositionIK(poses_wrt_base[0], seed, timeout, solution, error_code);

////    ROS_INFO("Pose: %f %f %f",poses_wrt_base[0].position.x, poses_wrt_base[0].position.y, poses_wrt_base[0].position.z);

////    ROS_INFO("Orient: %f %f %f %f",poses_wrt_base[0].orientation.x, poses_wrt_base[0].orientation.y,
////              poses_wrt_base[0].orientation.z, poses_wrt_base[0].orientation.w);
//    if(found_result)
//    {
//      found_success++;
//      bool result = ik_plugin_test.m_pKinematicsSolver->getPositionIK(poses_wrt_base[0], seed, solution, error_code);

//      if(result)
//      {
//        std::vector<geometry_msgs::Pose> new_poses;
//        new_poses.resize(1);
//        result_fk = ik_plugin_test.m_pKinematicsSolver->getPositionFK(fk_names, solution, new_poses);
//         if (isTwoPoseEqual(poses_wrt_world[0], new_poses[0]))
//         {
//          correct_success++;
//        }
//      }
//    }
//    else
//    {
//      ROS_INFO("The random joints angle results in IK failure: \n");
//      for (int i = 0; i < fk_values.size(); i++)
//      {
//        ROS_INFO_STREAM(fk_values[i] << "\n");
//      }
//      davinci_kinematics::Forward davinci_fk;
//      davinci_kinematics::Vectorq7x1 q_vec;
//      // copy failed to find ik solution joint list
//      for (int i = 0; i < fk_values.size(); i++)
//      {
//        q_vec(i) = fk_values[i];
//      }

//      Eigen::Affine3d affine_fk_pose_wrt_base = davinci_fk.fwd_kin_solve(q_vec);

////      Eigen::Affine3d affine_base_wrt_world = kinematic_state->getFrameTransform("PSM1_psm_base_link");

////      Eigen::Affine3d affine_fk_pose_wrt_world = affine_base_wrt_world * affine_fk_pose_wrt_base;

//      geometry_msgs::Pose fk_pose_wrt_base;

//      tf::poseEigenToMsg(affine_fk_pose_wrt_base, fk_pose_wrt_base);

//      davinci_kinematics::Inverse davinci_ik;

//      int ik_solve_return_val = davinci_ik.ik_solve(affine_fk_pose_wrt_base);

//      switch(ik_solve_return_val)
//      {
//        case -6 :
//        {
////          ROS_INFO("davinci ik solver has no solution");
//          for (int i = 0; i < fk_values.size(); i++)
//          {
//            output << fk_values[i] << "\n";
//          }
//          output << "\n";
//          break;
//        }
//        case 1 :
//        {
//          ROS_INFO("davinci ik solver has one solution");
////          std::vector<double> solution_ik;
////          convertVectorq7x1ToStdVector(davinci_inverse_.get_soln(), solution_ik);
////          solution.push_back(solution_ik);
//          found_success++;
////          correct_success++;
//          break;
//        }

//        default:
//        {
//          ROS_INFO("davinci ik solver has multiple solutions which is not allowed");
//          break;
//        }
//      }

////      bool result = ik_plugin_test.kinematics_solver->getPositionIK(fk_pose_wrt_base, seed, solution, error_code);


////      if(result)
////      {
////        std::vector<geometry_msgs::Pose> new_poses;
////        new_poses.resize(1);
//////        result_fk = ik_plugin_test.kinematics_solver->getPositionFK(fk_names, solution, new_poses);
//////        if (isTwoPoseEqual(fk_pose_wrt_world, new_poses[0]))
//////        {
//////           found_success++;
//////           correct_success++;
//////        }
////      }


//    }
////    EXPECT_NEAR(poses_wrt_world[0].position.x, new_poses[0].position.x, IK_NEAR_TRANSLATE);
////    EXPECT_NEAR(poses_wrt_world[0].position.y, new_poses[0].position.y, IK_NEAR_TRANSLATE);
////    EXPECT_NEAR(poses_wrt_world[0].position.z, new_poses[0].position.z, IK_NEAR_TRANSLATE);

////    EXPECT_NEAR(poses_wrt_world[0].orientation.x, new_poses[0].orientation.x, IK_NEAR_ORIENTATION);
////    EXPECT_NEAR(poses_wrt_world[0].orientation.y, new_poses[0].orientation.y, IK_NEAR_ORIENTATION);
////    EXPECT_NEAR(poses_wrt_world[0].orientation.z, new_poses[0].orientation.z, IK_NEAR_ORIENTATION);
////    EXPECT_NEAR(poses_wrt_world[0].orientation.w, new_poses[0].orientation.w, IK_NEAR_ORIENTATION);
//  }
//  output << "There are " << (number_ik_tests-found_success) << " sets of joint values failed to have IK solution." << "\n";
//  output.close();
//  ROS_INFO("IK Found Success Rate: %f",(double)found_success/number_ik_tests);
//  ROS_INFO("IK Found and Correct Success Rate: %f",(double)correct_success/found_success);
//  bool success_count = (found_success > 0.99 * number_ik_tests);
//  EXPECT_TRUE(success_count);
//  ROS_INFO("Elapsed time: %f", (ros::WallTime::now()-start_time).toSec());
//}

//
//TEST(ArmIKPlugin, searchIKWithCallbacks)
//{
//
//}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_davinci_moveit_kinematics_plugin");
  ros::NodeHandle nh;
  ros::Duration(3.0).sleep();
  return RUN_ALL_TESTS();
}
