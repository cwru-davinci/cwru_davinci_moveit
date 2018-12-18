//
// Created by sulu on 12/13/18.
//

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_TEST_HYBRID_MOTION_VALIDATOR_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_TEST_HYBRID_MOTION_VALIDATOR_H

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
  return std::make_shared<HybridValidStateSampler>(si);
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
                                                             "needle_r", "psm_one");

  std::vector <cwru_davinci_grasp::GraspInfo> grasp_pose = simpleGrasp.getAllPossibleNeedleGrasps();

//  geometry_msgs::PoseStamped = simpleGrasp.getNeedlePose();

  // create an instance of state space
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_pose.size(), grasp_pose));

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  si->setStateValidityChecker(std::make_shared<HybridStateValidityChecker>(node_handle, node_handle_priv, robot_name, object_name, si));
// or this call:
//  si->setStateValidityCheckingResolution(0.03); // 3%
  si->setMotionValidator(std::make_shared<HybridMotionValidator>(node_handle, node_handle_priv, robot_name, object_name, si));

  si->setValidStateSamplerAllocator(allocHybridValidStateSampler);

  ompl::base::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.2);
  se3_xyz_bounds.setHigh(0, 0.2);
  se3_xyz_bounds.setLow(1, -0.1);
  se3_xyz_bounds.setHigh(1, 0.1);
  se3_xyz_bounds.setLow(2, 0.3);
  se3_xyz_bounds.setHigh(2, 0.5);

  hystsp->setSE3Bounds(se3_xyz_bounds);

  si->setup();



  // check motion when ArmAndGraspDiffPoseSame
  // create a random state1
  ob::ScopedState<HybridObjectStateSpace> s1(hystsp);
  // create a random state2
  ob::ScopedState<HybridObjectStateSpace> s2(hystsp);

  ob::ScopedState<HybridObjectStateSpace> cstate(hystsp);

  while(!si->isValid(s1.get()) && !si->isValid(s2.get()))
  {
    s1.random();
    s2.random();
  }

  hystsp->as<ompl::base::SE3StateSpace>(0)->copyState(&(s1->se3State()), &(s2->se3State()));
  hystsp->interpolate(s1.get(), s2.get(), 0.5, cstate.get());

  si->checkMotion(s1.get(), cstate.get());

  return 0;
}



#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_TEST_HYBRID_MOTION_VALIDATOR_H
