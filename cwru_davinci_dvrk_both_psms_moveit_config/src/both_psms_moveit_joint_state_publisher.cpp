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
 * Desc: This node is meant to combine two subscribed sensor_msgs/JointState into one single
 * sensor_msgs/JointState.
 */

#ifndef CWRU_DAVINCI_DVRK_BOTH_PSMS_MOVEIT_CONFIG_BOTH_PSMS_MOVEIT_JOINT_STATE_PUBLISHER_
#define CWRU_DAVINCI_DVRK_BOTH_PSMS_MOVEIT_CONFIG_BOTH_PSMS_MOVEIT_JOINT_STATE_PUBLISHER_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class BothPSMsMoveitJointStatePublisher{
public:
  BothPSMsMoveitJointStatePublisher(const ros::NodeHandle& nh);

  ~BothPSMsMoveitJointStatePublisher();

  void publishBothPSMsJoinState();

private:

  void initializeSubscribers();

  void initializePublishers();

  void psmOneJointStateCallback(const sensor_msgs::JointState& dvrk_PSM1_joint_state);

  void psmTwoJointStateCallback(const sensor_msgs::JointState& dvrk_PSM2_joint_state);

  ros::NodeHandle nh_;

  sensor_msgs::JointState both_psms_jst_;
  sensor_msgs::JointState psm_one_jst_;
  sensor_msgs::JointState psm_two_jst_;

  ros::Subscriber psm_one_joint_state_sub_;
  ros::Subscriber psm_two_joint_state_sub_;
  ros::Publisher  both_psms_joint_state_pub_;
};

BothPSMsMoveitJointStatePublisher::BothPSMsMoveitJointStatePublisher(const ros::NodeHandle& nh) : nh_(nh)
{
  initializeSubscribers();
  initializePublishers();
}

BothPSMsMoveitJointStatePublisher::~BothPSMsMoveitJointStatePublisher()
{
  // blank
}

void BothPSMsMoveitJointStatePublisher::publishBothPSMsJoinState()
{
  if(!psm_one_jst_.name.empty() && !psm_two_jst_.name.empty())
  {
    both_psms_jst_.name.clear();
    both_psms_jst_.name.insert(both_psms_jst_.name.end(), psm_one_jst_.name.begin(), psm_one_jst_.name.end());
    both_psms_jst_.name.insert(both_psms_jst_.name.end(), psm_two_jst_.name.begin(), psm_two_jst_.name.end());
  }

  if(!psm_one_jst_.position.empty() && !psm_two_jst_.position.empty())
  {
    both_psms_jst_.position.clear();
    both_psms_jst_.position.insert(both_psms_jst_.position.end(), psm_one_jst_.position.begin(), psm_one_jst_.position.end());
    both_psms_jst_.position.insert(both_psms_jst_.position.end(), psm_two_jst_.position.begin(), psm_two_jst_.position.end());
  }

  if(!psm_one_jst_.velocity.empty() && !psm_two_jst_.velocity.empty())
  {
    both_psms_jst_.velocity.clear();
    both_psms_jst_.velocity.insert(both_psms_jst_.velocity.end(), psm_one_jst_.velocity.begin(), psm_one_jst_.velocity.end());
    both_psms_jst_.velocity.insert(both_psms_jst_.velocity.end(), psm_two_jst_.velocity.begin(), psm_two_jst_.velocity.end());
  }

  if(!psm_one_jst_.effort.empty() && !psm_two_jst_.effort.empty())
  {
    both_psms_jst_.effort.clear();
    both_psms_jst_.effort.insert(both_psms_jst_.effort.end(), psm_one_jst_.effort.begin(), psm_one_jst_.effort.end());
    both_psms_jst_.effort.insert(both_psms_jst_.effort.end(), psm_two_jst_.effort.begin(), psm_two_jst_.effort.end());
  }

  both_psms_jst_.header.stamp = ros::Time::now();

  both_psms_joint_state_pub_.publish(both_psms_jst_);

}

void BothPSMsMoveitJointStatePublisher::initializePublishers()
{
  ROS_INFO("Initializing Publishers");
  both_psms_joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 1, true);
}

void BothPSMsMoveitJointStatePublisher::initializeSubscribers()
{
  ROS_INFO("Initializing Services");
  psm_one_joint_state_sub_ = nh_.subscribe("/dvrk/PSM1/joint_states", 1,
                                           &BothPSMsMoveitJointStatePublisher::psmOneJointStateCallback, this);

  psm_two_joint_state_sub_ = nh_.subscribe("/dvrk/PSM2/joint_states", 1,
                                           &BothPSMsMoveitJointStatePublisher::psmTwoJointStateCallback, this);
}

void BothPSMsMoveitJointStatePublisher::psmOneJointStateCallback(const sensor_msgs::JointState& dvrk_PSM1_joint_state)
{
  psm_one_jst_.name.clear();
  psm_one_jst_.position.clear();
  psm_one_jst_.velocity.clear();
  psm_one_jst_.effort.clear();

  for (int i = 0; i < dvrk_PSM1_joint_state.name.size(); ++i)
  {
    std::string jnt_name = "PSM1/" + dvrk_PSM1_joint_state.name[i];
    psm_one_jst_.name.push_back(jnt_name);
  }

  for(int i = 0; i < dvrk_PSM1_joint_state.position.size(); ++i)
  {
    psm_one_jst_.position.push_back(dvrk_PSM1_joint_state.position[i]);
  }

  for(int i = 0; i < dvrk_PSM1_joint_state.velocity.size(); ++i)
  {
    psm_one_jst_.velocity.push_back(dvrk_PSM1_joint_state.velocity[i]);
  }

  for(int i = 0; i < dvrk_PSM1_joint_state.effort.size(); ++i)
  {
    psm_one_jst_.effort.push_back(dvrk_PSM1_joint_state.effort[i]);
  }

}

void BothPSMsMoveitJointStatePublisher::psmTwoJointStateCallback(const sensor_msgs::JointState& dvrk_PSM2_joint_state)
{
  psm_two_jst_.name.clear();
  psm_two_jst_.position.clear();
  psm_two_jst_.velocity.clear();
  psm_two_jst_.effort.clear();

  for (int i = 0; i < dvrk_PSM2_joint_state.name.size(); ++i)
  {
    std::string jnt_name = "PSM2/" + dvrk_PSM2_joint_state.name[i];
    psm_two_jst_.name.push_back(jnt_name);
  }

  for(int i = 0; i < dvrk_PSM2_joint_state.position.size(); ++i)
  {
    psm_two_jst_.position.push_back(dvrk_PSM2_joint_state.position[i]);
  }

  for(int i = 0; i < dvrk_PSM2_joint_state.velocity.size(); ++i)
  {
    psm_two_jst_.velocity.push_back(dvrk_PSM2_joint_state.velocity[i]);
  }

  for(int i = 0; i < dvrk_PSM2_joint_state.effort.size(); ++i)
  {
    psm_two_jst_.effort.push_back(dvrk_PSM2_joint_state.effort[i]);
  }

}

int main(int argc, char** argv)
{
  // ROS set-ups:
  ros::init(argc, argv, "both_psms_moveit_joint_state_publisher");

  ros::NodeHandle nh;

  ROS_INFO("main: instantiating an object of type BothPSMsMoveitJointStatePublisher");
  BothPSMsMoveitJointStatePublisher bothPSMsMoveitJointStatePublisher(nh);

  ROS_INFO("main: going into spin; let the callbacks do all the work");
  while(ros::ok())
  {
    bothPSMsMoveitJointStatePublisher.publishBothPSMsJoinState();

    ros::spinOnce();

    ros::Duration(0.1).sleep();
  }
  return 0;
}

#endif