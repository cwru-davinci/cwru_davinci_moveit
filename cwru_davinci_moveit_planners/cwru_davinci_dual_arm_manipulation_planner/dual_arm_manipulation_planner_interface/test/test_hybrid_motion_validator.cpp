//
// Created by sulu on 12/13/18.
//
#include <ros/ros.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <moveit/dual_arm_manipulation_planner_interface//hybrid_motion_validator.h>
#include <moveit/dual_arm_manipulation_planner_interface/hybrid_valid_state_sampler.h>
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>


namespace ob = ompl::base;
//namespace og = ompl::geometric;
using namespace dual_arm_manipulation_planner_interface;


// return an sampler
ob::ValidStateSamplerPtr allocHybridValidStateSampler(const ob::SpaceInformation *si)
{

  // we can perform any additional setup / configuration of a sampler here,
  // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
  return std::make_shared<HybridValidStateSampler>("robot_description", si);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_hybrid_motion_validator");
  ros::AsyncSpinner spinner(1);
  ros::Duration(3.0).sleep();
  spinner.start();
//  testing::InitGoogleTest(&argc, argv);
//  return RUN_ALL_TESTS();


  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");
  std::string object_name = "needle_r";
  std::string robot_name = "robot_description";
  cwru_davinci_grasp::DavinciSimpleNeedleGrasper simpleGrasp(node_handle,
                                                             node_handle_priv,
                                                             "psm_one", "needle_r");

  std::vector <cwru_davinci_grasp::GraspInfo> grasp_pose = simpleGrasp.getAllPossibleNeedleGrasps();

  // create an instance of state space
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_pose.size(), grasp_pose));

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ompl::base::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.3);
  se3_xyz_bounds.setHigh(0, 0.3);
  se3_xyz_bounds.setLow(1, -0.05);
  se3_xyz_bounds.setHigh(1, 0.05);
  se3_xyz_bounds.setLow(2, 0.3);
  se3_xyz_bounds.setHigh(2, 0.5);

  hystsp->setSE3Bounds(se3_xyz_bounds);

  si->setValidStateSamplerAllocator(allocHybridValidStateSampler);

  si->setStateValidityChecker(std::make_shared<HybridStateValidityChecker>(node_handle, robot_name, object_name, si));
// or this call:
//  si->setStateValidityCheckingResolution(0.03); // 3%
  si->setMotionValidator(std::make_shared<HybridMotionValidator>(node_handle, node_handle_priv, robot_name, object_name, si));

  si->setup();

  // check motion when ArmAndGraspDiffPoseSame
  // create a random state1
  ob::ScopedState<HybridObjectStateSpace> s1(hystsp);
  // create a random state2
  ob::ScopedState<HybridObjectStateSpace> s2(hystsp);

  ob::ScopedState<HybridObjectStateSpace> cstate(hystsp);

  ompl::base::ValidStateSamplerPtr sampler = si->allocValidStateSampler();

  std::vector<double> s1_needle_pose_translation;
  std::vector<double> s1_needle_pose_orientation;
  std::vector<double> cs_needle_pose_translation;
  std::vector<double> cs_needle_pose_orientation;
  std::vector<double> s1_joint_values;
  std::vector<double> cs_joint_values;
  int s1_arm_index;
  int s1_grasp_pose_index;
  int cs_arm_index;
  int cs_grasp_pose_index;

  if (node_handle_priv.hasParam("s1_needle_pose_translation"))
  {
    XmlRpc::XmlRpcValue needle_pose_list;
    node_handle_priv.getParam("s1_needle_pose_translation", needle_pose_list);

    ROS_ASSERT(needle_pose_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_pose_list.size(); ++i)
    {
      ROS_ASSERT(needle_pose_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      s1_needle_pose_translation.push_back(static_cast<double>(needle_pose_list[i]));
    }
  }

  if (node_handle_priv.hasParam("s1_needle_pose_orientation"))
  {
    XmlRpc::XmlRpcValue needle_ori_list;
    node_handle_priv.getParam("s1_needle_pose_orientation", needle_ori_list);

    ROS_ASSERT(needle_ori_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_ori_list.size(); ++i)
    {
      ROS_ASSERT(needle_ori_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      s1_needle_pose_orientation.push_back(static_cast<double>(needle_ori_list[i]));
    }
  }

  if (node_handle_priv.hasParam("cs_needle_pose_translation"))
  {
    XmlRpc::XmlRpcValue needle_pose_list;
    node_handle_priv.getParam("cs_needle_pose_translation", needle_pose_list);

    ROS_ASSERT(needle_pose_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_pose_list.size(); ++i)
    {
      ROS_ASSERT(needle_pose_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      cs_needle_pose_translation.push_back(static_cast<double>(needle_pose_list[i]));
    }
  }

  if (node_handle_priv.hasParam("cs_needle_pose_orientation"))
  {
    XmlRpc::XmlRpcValue needle_ori_list;
    node_handle_priv.getParam("cs_needle_pose_orientation", needle_ori_list);

    ROS_ASSERT(needle_ori_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_ori_list.size(); ++i)
    {
      ROS_ASSERT(needle_ori_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      cs_needle_pose_orientation.push_back(static_cast<double>(needle_ori_list[i]));
    }
  }

  if (node_handle_priv.hasParam("s1_joint_values"))
  {
    XmlRpc::XmlRpcValue joint_value_list;
    node_handle_priv.getParam("s1_joint_values", joint_value_list);

    ROS_ASSERT(joint_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < joint_value_list.size(); ++i)
    {
      ROS_ASSERT(joint_value_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      s1_joint_values.push_back(static_cast<double>(joint_value_list[i]));
    }
  }

  if (node_handle_priv.hasParam("cs_joint_values"))
  {
    XmlRpc::XmlRpcValue joint_value_list;
    node_handle_priv.getParam("cs_joint_values", joint_value_list);

    ROS_ASSERT(joint_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < joint_value_list.size(); ++i)
    {
      ROS_ASSERT(joint_value_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      cs_joint_values.push_back(static_cast<double>(joint_value_list[i]));
    }
  }

  node_handle_priv.getParam("s1_arm_index", s1_arm_index);
  node_handle_priv.getParam("s1_grasp_pose_index", s1_grasp_pose_index);
  node_handle_priv.getParam("cs_arm_index", cs_arm_index);
  node_handle_priv.getParam("cs_grasp_pose_index", cs_grasp_pose_index);

  s1->se3State().setXYZ(s1_needle_pose_translation[0],
                        s1_needle_pose_translation[1],
                        s1_needle_pose_translation[2]);
  s1->se3State().rotation().setAxisAngle(s1_needle_pose_orientation[0],
                                         s1_needle_pose_orientation[1],
                                         s1_needle_pose_orientation[2],
                                         s1_needle_pose_orientation[3]);
  s1->armIndex().value = s1_arm_index;  // set arm index
  s1->graspIndex().value = s1_grasp_pose_index;  // set grasp index

  bool valid_s1 = si->isValid(s1.get());

  cstate->se3State().setXYZ(cs_needle_pose_translation[0],
                            cs_needle_pose_translation[1],
                            cs_needle_pose_translation[2]);
  cstate->se3State().rotation().setAxisAngle(cs_needle_pose_orientation[0],
                                             cs_needle_pose_orientation[1],
                                             cs_needle_pose_orientation[2],
                                             cs_needle_pose_orientation[3]);
  cstate->armIndex().value = cs_arm_index;  // set arm index
  cstate->graspIndex().value = cs_grasp_pose_index;  // set grasp index
  bool valid_cstate = si->isValid(cstate.get());

//  bool valid_s1 = sampler->sample(s1.get());
//  bool valid_s2 = sampler->sample(s2.get());
//
//  bool valid_cstate = false;
//
//  while(!valid_s1 || !valid_s2 || !valid_cstate)
//  {
//    valid_s1 = sampler->sample(s1.get());
//    valid_s2 = sampler->sample(s2.get());
//    if(valid_s1 && valid_s2)
//    {
//      hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//      valid_cstate = si->isValid(cstate.get());
//      if(valid_cstate)
//      {
//        break;
//      }
//    }
//  }

//  bool valid_cstate = false;
//  while(!valid_cstate)
//  {
//    if(valid_s1)
//    {
//      hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());
//      valid_cstate = si->isValid(cstate.get());
//      valid_s1 = sampler->sample(s1.get());
//      sampler->sample(s2.get());
//    }
//    else
//    {
//      valid_s1 = sampler->sample(s1.get());
//    }
//  }

  std::cout << "valid s1:" << std::endl;
  hystsp->printState(s1.get(), std::cout);
  std::cout << "valid s2:" << std::endl;
  hystsp->printState(s2.get(), std::cout);
  std::cout << "valid cstate:" << std::endl;
  hystsp->printState(cstate.get(), std::cout);

  si->checkMotion(s1.get(), cstate.get());

  return 0;
}