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

#include <pluginlib/class_list_macros.h>
#include <cwru_davinci_moveit_kinematics_plugin/davinci_moveit_kinematics_plugin.h>
#include <moveit/kinematics_base/kinematics_base.h>


//register DavinciMoveitKinematicsPlugin as a KinematicsBase implementation
PLUGINLIB_EXPORT_CLASS(davinci_moveit_kinematics::DavinciMoveitKinematicsPlugin, kinematics::KinematicsBase);

namespace davinci_moveit_kinematics
{

  DavinciMoveitKinematicsPlugin::DavinciMoveitKinematicsPlugin() : active_(false)
  {}

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

  bool DavinciMoveitKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                                    const std::vector<double> &ik_seed_state,
                                                    std::vector<double> &solution,
                                                    moveit_msgs::MoveItErrorCodes &error_code,
                                                    const kinematics::KinematicsQueryOptions &options) const
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    KDL::Frame pose_desired;
    tf::poseMsgToKDL(ik_pose, pose_desired);

    // Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i = 0; i < dimension_; i++)
    {
      jnt_pos_in(i) = ik_seed_state[i];
    }

    int ik_valid = davinci_moveit_ik_solver_ptr_->CartToJnt(jnt_pos_in,
                                                            pose_desired,
                                                            jnt_pos_out);

    if(ik_valid == davinci_moveit_kinematics::NO_IK_SOLUTION)
    {
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    if(ik_valid >= 0)
    {
      solution.resize(dimension_);
      for(int i = 0; i < dimension_; i++)
      {
        solution[i] = jnt_pos_out(i);
      }
      error_code.val = error_code.SUCCESS;
      return true;
    }
    else
    {
      ROS_INFO("An IK solution could not be found");
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }
  }

  bool DavinciMoveitKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                       const std::vector<double> &ik_seed_state,
                                                       double timeout,
                                                       std::vector<double> &solution,
                                                       moveit_msgs::MoveItErrorCodes &error_code,
                                                       const kinematics::KinematicsQueryOptions &options) const
  {
    static kinematics::KinematicsBase::IKCallbackFn solution_callback = NULL;
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback,
                            error_code);
  }

  bool DavinciMoveitKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                       const std::vector<double> &ik_seed_state,
                                                       double timeout,
                                                       const std::vector<double> &consistency_limits,
                                                       std::vector<double> &solution,
                                                       moveit_msgs::MoveItErrorCodes &error_code,
                                                       const kinematics::KinematicsQueryOptions &options) const
  {
    static IKCallbackFn solution_callback = NULL;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback,
                            error_code);
  }

  bool DavinciMoveitKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                       const std::vector<double> &ik_seed_state,
                                                       double timeout,
                                                       std::vector<double> &solution,
                                                       const IKCallbackFn &solution_callback,
                                                       moveit_msgs::MoveItErrorCodes &error_code,
                                                       const kinematics::KinematicsQueryOptions &options) const
  {
    static std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback,
                            error_code);
  }

  bool DavinciMoveitKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
                                                       const std::vector<double> &ik_seed_state,
                                                       double timeout,
                                                       const std::vector<double> &consistency_limits,
                                                       std::vector<double> &solution,
                                                       const IKCallbackFn &solution_callback,
                                                       moveit_msgs::MoveItErrorCodes &error_code,
                                                       const kinematics::KinematicsQueryOptions &options) const
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
      error_code.val = error_code.FAILURE;
      return false;
    }
    if(!consistency_limits.empty() && consistency_limits.size() != dimension_)
    {
      ROS_ERROR("Consistency limits should be of size: %d", dimension_);
      error_code.val = error_code.FAILURE;
    }

    KDL::Frame pose_desired;
    tf::poseMsgToKDL(ik_pose, pose_desired);

    // Do the IK
    KDL::JntArray jnt_pos_in;
    KDL::JntArray jnt_pos_out;
    jnt_pos_in.resize(dimension_);
    for(int i = 0; i < dimension_; i++)
    {
      jnt_pos_in(i) = ik_seed_state[i];
    }

    int ik_valid;
    if(consistency_limits.empty())
    {
      ik_valid = davinci_moveit_ik_solver_ptr_->CartToJntSearch(jnt_pos_in,
                                                                pose_desired,
                                                                jnt_pos_out,
                                                                timeout,
                                                                error_code,
                                                                solution_callback ?
                                                                boost::bind(solution_callback, _1, _2, _3) :
                                                                IKCallbackFn());
    }
    else
    {
      ik_valid = davinci_moveit_ik_solver_ptr_->CartToJntSearch(jnt_pos_in,
                                                                pose_desired,
                                                                jnt_pos_out,
                                                                timeout,
                                                                consistency_limits[0],
                                                                error_code,
                                                                solution_callback ?
                                                                boost::bind(solution_callback, _1, _2, _3) :
                                                                IKCallbackFn());
    }

    if(ik_valid == davinci_moveit_kinematics::NO_IK_SOLUTION)
    {
      return false;
    }

    if(ik_valid >= 0)
    {
      solution.resize(dimension_);
      for(int i = 0; i < dimension_; i++)
      {
        solution[i] = jnt_pos_out(i);
      }
      return true;
    }
    else
    {
      ROS_INFO("An IK solution could not be found");
      return false;
    }
  }

  bool DavinciMoveitKinematicsPlugin::getPositionFK(const std::vector <std::string> &link_names,
                                                    const std::vector<double> &joint_angles,
                                                    std::vector <geometry_msgs::Pose> &poses) const
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
      return false;
    }

    KDL::Frame p_out;
    KDL::JntArray jnt_pos_in;
    geometry_msgs::PoseStamped pose;
    tf::Stamped<tf::Pose> tf_pose;

    jnt_pos_in.resize(dimension_);
//    ROS_INFO("Numebr of rows in KDL::JntArrary object jnt_pos_in: %d", jnt_pos_in.rows());
    for(int i=0; i < dimension_; i++)
    {
      jnt_pos_in(i) = joint_angles[i];
//      ROS_ERROR("Joint angle: %d %f",i,jnt_pos_in(i));
    }

//    ROS_ERROR("Number of joints %d", kdl_chain_.getNrOfJoints());

    poses.resize(link_names.size());
    bool valid = true;
    for(unsigned int i=0; i < poses.size(); i++)
    {
//      ROS_INFO("End effector index: %d",davinci_moveit_kinematics::getKDLSegmentIndex(kdl_chain_,link_names[i]));
      if(jnt_to_pose_solver_ptr_->
        JntToCart(jnt_pos_in, p_out, davinci_moveit_kinematics::getKDLSegmentIndex(kdl_chain_, link_names[i])) >= 0)
      {
        tf::poseKDLToMsg(p_out, poses[i]);
      }
      else
      {
        ROS_ERROR("Could not compute FK for %s", link_names[i].c_str());
        valid = false;
      }
    }
    return valid;
  }

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
    kinematics::KinematicsBase::setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
    urdf::Model robot_model;
    std::string xml_string;
    ros::NodeHandle private_handle("~/" + group_name);
    dimension_ = davinci_moveit_kinematics::NUM_JOINTS_ARM7DOF;  // TODO left to decide

    while(!davinci_moveit_kinematics::loadRobotModel(private_handle, robot_model, xml_string) && private_handle.ok())
    {
      ROS_ERROR("Could not load robot model. Are you sure the robot model is on the parameter server?");
      ros::Duration(0.5).sleep();
    }

    ROS_INFO("Loading KDL Tree");
    if(!davinci_moveit_kinematics::getKDLChain(xml_string, base_frame, tip_frame, kdl_chain_))
    {
      active_ = false;
      ROS_ERROR("Could not load KDL tree");
    }

    ROS_INFO("Advertising services");
    jnt_to_pose_solver_ptr_.reset(
      new KDL::ChainFkSolverPos_recursive(kdl_chain_));  // reset() is a member method of boost::shared_ptr class
//    private_handle.param<int>("free_angle", free_angle_, 2);  // TODO, need to change

    // TODO, need to implement this reset function in davinci_kinematics library
    davinci_moveit_ik_solver_ptr_.reset(
      new davinci_moveit_kinematics::DavinciMoveitIKSolver(robot_model, base_frame, tip_frame, search_discretization));

    if(!davinci_moveit_ik_solver_ptr_->active_)
    {
      ROS_ERROR("Could not load IK solver");
      active_ = false;
    }
    else
    {
      davinci_moveit_ik_solver_ptr_->getSolverInfo(ik_solver_info_);
      davinci_moveit_kinematics::getKDLChainInfo(kdl_chain_, fk_solver_info_);
      ROS_INFO("Number of joints in KDL::Chain object kdl_chain_: %d", kdl_chain_.getNrOfJoints());

//      bool exit_value;
//      KDL::Segment root_segment;
//      KDL::Segment leaf_segment;
//      exit_value = kdl_chain_.getSegment(0);

      fk_solver_info_.joint_names = ik_solver_info_.joint_names;

      for(unsigned int i = 0; i < ik_solver_info_.joint_names.size(); i++)
      {
        ROS_INFO("daVinciKinematics:: joint name: %s", ik_solver_info_.joint_names[i].c_str());
      }
      for(unsigned int i = 0; i < ik_solver_info_.link_names.size(); i++)
      {
        ROS_INFO("daVinciKinematics can solve IK for %s", ik_solver_info_.link_names[i].c_str());
      }
      for(unsigned int i = 0; i < fk_solver_info_.link_names.size(); i++)
      {
        ROS_INFO("daVinciKinematics can solve FK for %s", fk_solver_info_.link_names[i].c_str());
      }
      ROS_INFO("DavinciMoveitKinematicsPlugin::active for %s", group_name.c_str());
      active_ = true;
    }

//    davinci_moveit_ik_solver_ptr_->setFreeAngle(2);  // TODO
    return active_;
  }

  /**
    * @brief Return all the joint names in the order they are used internally
    */
  const std::vector<std::string>& DavinciMoveitKinematicsPlugin::getJointNames() const
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
    }
    return ik_solver_info_.joint_names;
  }

  /**
    * @brief Return all the link names in the order they are represented internally
    */
  const std::vector<std::string>& DavinciMoveitKinematicsPlugin::getLinkNames() const
  {
    if(!active_)
    {
      ROS_ERROR("kinematics not active");
    }
    return fk_solver_info_.link_names;
  }

}  // namespace


