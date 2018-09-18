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
#include <cwru_davinci_moveit_kinematics_plugin/davinci_moveit_ik.h>
#include <moveit_msgs/KinematicSolverInfo.h>

namespace davinci_moveit_kinematics
{
  DavinciMoveitIK::DavinciMoveitIK()
  {
  }

  DavinciMoveitIK::~DavinciMoveitIK()
  {
  }

  bool DavinciMoveitIK::init(const urdf::Model &robot_model, const std::string &root_name, const std::string &tip_name)
  {
    std::vector<urdf::Pose> link_offset;
    int num_joints = 0;
    boost::shared_ptr<const urdf::Link> link = robot_model.getLink(tip_name);  // get a link ptr its name

    // After while block executed, the link_offset list will be populated with transforms from each link to its child link
    while(link && num_joints < davinci_moveit_kinematics::NUM_JOINTS_ARM7DOF)
    {
      boost::shared_ptr<const urdf::Joint> joint;
      boost::shared_ptr<urdf::Joint> parent_joint = link->parent_joint;  // get joint ptr by current link's parent_joint

      if(parent_joint)
      {
        joint = robot_model.getJoint(parent_joint->name);
        ROS_INFO("Name of added joints: %s", joint->name.c_str());
      }
      if(!joint)
      {
        if(parent_joint)
        {
          ROS_ERROR("Could not find joint: %s", parent_joint->name.c_str());
        }
        else
        {
          ROS_ERROR("Link %s has no parent joint", link->name.c_str());
        }
        return false;
      }

      if(joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
//      if(joint->type != urdf::Joint::UNKNOWN)
      {
        // get transform from Parent Link frame to Joint frame
        link_offset.push_back(parent_joint->parent_to_joint_origin_transform);

        // TODO the following list may not be used
//        angle_multipliers_.push_back(joint->axis.x * fabs(joint->axis.x) + joint->axis.y * fabs(joint->axis.y) +
//                                     joint->axis.z * fabs(joint->axis.z));
//        ROS_INFO("Joint axis: %d, %f, %f, %f",7-num_joints,joint->axis.x,joint->axis.y,joint->axis.z);

        // The if-else block checks joint type and store the joint's limit in both min_angles_ and max_angles_ lists
        if(joint->type != urdf::Joint::CONTINUOUS)
        {
          if(joint->safety)
          {
            min_angles_.push_back(joint->safety->soft_lower_limit);
            max_angles_.push_back(joint->safety->soft_upper_limit);
          }
          else
          {
            if(joint->limits)
            {
              min_angles_.push_back(joint->limits->lower);
              max_angles_.push_back(joint->limits->upper);
            }
            else
            {
              min_angles_.push_back(0.0);
              max_angles_.push_back(0.0);
              ROS_WARN("No joint limits for joint '%s'", joint->name.c_str());
            }
          }
          continuous_joint_.push_back(false);
        }
        else
        {
          min_angles_.push_back(-M_PI);
          max_angles_.push_back(M_PI);
          continuous_joint_.push_back(true);
        }
        addJointToChainInfo(parent_joint, solver_info_);
        num_joints++;
      }
      link = robot_model.getLink(link->getParent()->name);  // assign current link with its parent link
    }

    solver_info_.link_names.push_back(tip_name);

    // We expect order from root to tip, so reverse the order
//    std::reverse(angle_multipliers_.begin(),angle_multipliers_.end());
    std::reverse(min_angles_.begin(), min_angles_.end());
    std::reverse(max_angles_.begin(), max_angles_.end());
    std::reverse(link_offset.begin(), link_offset.end());
    std::reverse(solver_info_.limits.begin(), solver_info_.limits.end());
    std::reverse(solver_info_.joint_names.begin(), solver_info_.joint_names.end());
    std::reverse(solver_info_.link_names.begin(), solver_info_.link_names.end());
    std::reverse(continuous_joint_.begin(), continuous_joint_.end());

    if(num_joints != NUM_JOINTS_ARM7DOF)
    {
      ROS_FATAL("DavinciIK:: Chain from %s to %s does not have %d joints", root_name.c_str(), tip_name.c_str(),
                NUM_JOINTS_ARM7DOF);
      return false;
    }

    // TODO the following variables is of no use for davinci IK solver
//    torso_shoulder_offset_x_ = link_offset[0].position.x;
//    torso_shoulder_offset_y_ = link_offset[0].position.y;
//    torso_shoulder_offset_z_ = link_offset[0].position.z;
//    shoulder_upperarm_offset_ = distance(link_offset[1]);
//    upperarm_elbow_offset_ = distance(link_offset[3]);
//    elbow_wrist_offset_ = distance(link_offset[5]);
//    shoulder_elbow_offset_ = shoulder_upperarm_offset_ + upperarm_elbow_offset_;
//    shoulder_wrist_offset_ = shoulder_upperarm_offset_+upperarm_elbow_offset_+elbow_wrist_offset_;

//    Eigen::Matrix4f home = Eigen::Matrix4f::Identity();
//    home(0,3) = shoulder_upperarm_offset_ +  upperarm_elbow_offset_ +  elbow_wrist_offset_;
//    home_inv_ = home.inverse();
//    grhs_ = home;
//    gf_ = home_inv_;
    solution_.resize(NUM_JOINTS_ARM7DOF);
    return true;
  }

  void DavinciMoveitIK::computeIKSolution(const Eigen::Affine3d &g_in, std::vector<std::vector<double>> &solution)
  {
    solution.clear();

//    ROS_INFO_STREAM("IK input goal pose translation: \n" << g_in.translation());
//    ROS_INFO_STREAM("IK input goal pose rotation: \n" << g_in.rotation());

    int ik_solve_return_val = davinci_inverse_.ik_solve(g_in);
//    ROS_INFO("IK solution return value %d", ik_solve_return_val);
    switch(ik_solve_return_val)
    {
      case -6 :
      {
//        ROS_INFO("davinci ik solver has no solution");
        //  davinci ik solver has no solution
        break;
      }
      case 1 :
      {
//        ROS_INFO("davinci ik solver has one solution");
        //  davinci ik solver has one solution
        std::vector<double> solution_ik;
        convertVectorq7x1ToStdVector(davinci_inverse_.get_soln(), solution_ik);
        solution.push_back(solution_ik);
        break;
      }

      default:
      {
        ROS_INFO("davinci ik solver has multiple solutions which is not allowed");
        break;
      }
    }
//    ROS_INFO("Out of switch");
  }

  void DavinciMoveitIK::getSolverInfo(moveit_msgs::KinematicSolverInfo &info)
  {
    info = solver_info_;
  }

  void DavinciMoveitIK::addJointToChainInfo(boost::shared_ptr<const urdf::Joint> joint,
                                            moveit_msgs::KinematicSolverInfo &info)
  {
    moveit_msgs::JointLimits limit;
    info.joint_names.push_back(joint->name);  // Joints are coming in reverse order

    ROS_INFO_STREAM("Added joints name %s " << joint->name.c_str() << " and its joint type %d " << joint->type);

    if(joint->type != urdf::Joint::CONTINUOUS)
    {
      if(joint->safety)
      {
        limit.min_position = joint->safety->soft_lower_limit;
        limit.max_position = joint->safety->soft_upper_limit;
        limit.has_position_limits = true;
      }
      else
      {
        if(joint->limits)
        {
          limit.min_position = joint->limits->lower;
          limit.max_position = joint->limits->upper;
          limit.has_position_limits = true;
        }
        else
        {
          limit.has_position_limits = false;
        }
      }
    }
    else
    {
      limit.min_position = -M_PI;
      limit.max_position = M_PI;
      limit.has_position_limits = false;
    }

    if(joint->limits)
    {
      limit.max_velocity = joint->limits->velocity;
      limit.has_velocity_limits = true;
    }
    else
    {
      limit.has_velocity_limits = false;
    }

    info.limits.push_back(limit);
    return;
  }

//  bool DavinciMoveitIK::checkJointLimits(const std::vector<double> &joint_values) const
//  {
//
//  }
//
//  bool DavinciMoveitIK::checkJointLimits(const double &joint_value, const int &joint_num) const
//  {
//
//  }

  void DavinciMoveitIK::convertVectorq7x1ToStdVector(const davinci_kinematics::Vectorq7x1 &vec_in,
                                                     std::vector<double> &vec_out) const
  {

    vec_out.clear();
    vec_out.resize(NUM_JOINTS_ARM7DOF, 0.0);
    for(int i = 0; i < vec_out.size(); i++)
    {
      vec_out[i] = vec_in(i);
    }
  }
}
