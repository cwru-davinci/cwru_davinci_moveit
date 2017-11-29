/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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

/* Author: Su Lu */

#include <cwru_davinci_moveit_kinematics_plugin/davinci_moveit_kinematics_plugin.h>
namespace davinci_moveit_kinematics_plugin
{
    /**
     *  @brief Specifies if the solver is active or not
     *  @return True if the solver is active, false otherwise.
     */
    bool DavinciMoveitKinematicsPlugin::isActive()
    {
        if(active_)
        {
            return true;
        }
        return false;
    }

//    bool DavinciMoveitKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
//                                                              const std::vector<double> &ik_seed_state,
//                                                              std::vector<double> &solution,
//                                                              moveit_msgs::MoveItErrorCodes &error_code,
//                                                              const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
//    {
//
//    }
//
//    bool DavinciMoveitKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
//                                                                 const std::vector<double> &ik_seed_state,
//                                                                 double timeout,
//                                                                 std::vector<double> &solution,
//                                                                 moveit_msgs::MoveItErrorCodes &error_code,
//                                                                 const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
//    {
//
//    }
//
//    bool DavinciMoveitKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
//                                                                 const std::vector<double> &ik_seed_state,
//                                                                 double timeout,
//                                                                 const std::vector<double> &consistency_limits,
//                                                                 std::vector<double> &solution,
//                                                                 moveit_msgs::MoveItErrorCodes &error_code,
//                                                                 const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
//    {
//
//    }
//
//    bool DavinciMoveitKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
//                                                                 const std::vector<double> &ik_seed_state,
//                                                                 double timeout,
//                                                                 std::vector<double> &solution,
//                                                                 const IKCallbackFn &solution_callback,
//                                                                 moveit_msgs::MoveItErrorCodes &error_code,
//                                                                 const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
//    {
//
//
//    }
//
//    bool DavinciMoveitKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
//                                                                 const std::vector<double> &ik_seed_state,
//                                                                 double timeout,
//                                                                 const std::vector<double> &consistency_limits,
//                                                                 std::vector<double> &solution,
//                                                                 const IKCallbackFn &solution_callback,
//                                                                 moveit_msgs::MoveItErrorCodes &error_code,
//                                                                 const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
//    {
//
//    }
//
//    bool DavinciMoveitKinematicsPlugin::getPositionFK(const std::vector <std::string> &link_names,
//                                                              const std::vector<double> &joint_angles,
//                                                              std::vector <geometry_msgs::Pose> &poses) const
//    {
//
//    }

    /**
     * @brief  Initialization function for the kinematics
     * @return True if initialization was successful, false otherwise
     */
    bool DavinciMoveitKinematicsPlugin::initialize(const std::string &robot_description,
                                                   const std::string &group_name,
                                                   const std::string &base_frame,
                                                   const std::string &tip_frame,
                                                   double search_discretization)
    {

        return active_;
    }

//    /**
//      * @brief Return all the joint names in the order they are used internally
//      */
//    const DavinciMoveitKinematicsPlugin::std::vector<std::string>& getJointNames() const
//    {
//
//    }
//
//    /**
//      * @brief Return all the link names in the order they are represented internally
//      */
//    const DavinciMoveitKinematicsPlugin::std::vector<std::string>& getLinkNames() const
//    {
//
//    }
}


