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

#ifndef DAVINCI_MOVEIT_KINEMATICS_PLUGIN_H
#define DAVINCI_MOVEIT_KINEMATICS_PLUGIN_H

#include <ros/ros.h>
#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <vector>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

#include <kdl/chainfksolverpos_recursive.hpp>

#include <tf_conversions/tf_kdl.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <boost/shared_ptr.hpp>

#include <urdf/model.h>

#include <cwru_davinci_moveit_kinematics_plugin/davinci_moveit_ik_solver.h>
#include <cwru_davinci_moveit_kinematics_plugin/davinci_moveit_kinematics_constants.h>

/** @brief Namespace for the DavinciMoveitKinematicsPlugin */
namespace davinci_moveit_kinematics
{
  class DavinciMoveitKinematicsPlugin : public kinematics::KinematicsBase
  {
  public:
    /**
     *  @brief Plugin-able interface to the davinci moveit kinematics plugin
     */
    DavinciMoveitKinematicsPlugin();

    /**
     *  @brief Specifies if the solver is active or not
     *  @return True if the solver is active, false otherwise.
     */
    bool isActive();

    virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                               const std::vector<double> &ik_seed_state,
                               std::vector<double> &solution,
                               moveit_msgs::MoveItErrorCodes &error_code,
                               const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                  const std::vector<double> &ik_seed_state,
                                  double timeout,
                                  const std::vector<double> &consistency_limits,
                                  std::vector<double> &solution,
                                  const IKCallbackFn &solution_callback,
                                  moveit_msgs::MoveItErrorCodes &error_code,
                                  const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

    /**
     * @brief Given a set of joint angles and a set of links, compute their pose.
     * @param link_names
     * @param joint_angles
     * @param poses
     * @return
     */
    virtual bool getPositionFK(const std::vector <std::string> &link_names,
                               const std::vector<double> &joint_angles,
                               std::vector <geometry_msgs::Pose> &poses) const;

    /**
     * @brief  Initialization function for the kinematics
     * @return True if initialization was successful, false otherwise
     */
    virtual bool initialize(const std::string &robot_description,
                            const std::string &group_name,
                            const std::string &base_frame,
                            const std::string &tip_frame,
                            double search_discretization);

    /**
     * @brief Return all the joint names in the order they are used internally
     */
    const std::vector<std::string>& getJointNames() const;

    /**
     * @brief Return all the link names in the order they are represented internally
     */
    const std::vector<std::string>& getLinkNames() const;

  protected:

    bool active_;
//    int free_angle_;
//    urdf::Model robot_model_;
//    ros::NodeHandle node_handle_, root_handle_;
    boost::shared_ptr<davinci_moveit_kinematics::DavinciMoveitIKSolver> davinci_moveit_ik_solver_ptr_;
    int dimension_;
    boost::shared_ptr<KDL::ChainFkSolverPos_recursive> jnt_to_pose_solver_ptr_;
    KDL::Chain kdl_chain_;
    moveit_msgs::KinematicSolverInfo ik_solver_info_, fk_solver_info_;
  };

}


#endif  // DAVINCI_MOVEIT_KINEMATICS_PLUGIN_H
