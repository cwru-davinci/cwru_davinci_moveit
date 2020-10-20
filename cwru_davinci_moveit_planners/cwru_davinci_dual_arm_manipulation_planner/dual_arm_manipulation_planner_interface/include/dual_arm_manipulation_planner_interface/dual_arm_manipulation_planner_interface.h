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
 * Description:
 */


#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DUAL_ARM_MANIPULATION_PLANNER_INTERFACE_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DUAL_ARM_MANIPULATION_PLANNER_INTERFACE_H

//#include <moveit/ompl_interface/planning_context_manager.h>
//#include <moveit/ompl_interface/constraints_library.h>
//#include <moveit/constraint_samplers/constraint_sampler_manager.h>
//#include <moveit/constraint_sampler_manager_loader/constraint_sampler_manager_loader.h>
//#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <string>
#include <map>
#include <ros/ros.h>

/**
 * @brief The MoveIt! interface to dual_arm_manipulation_planner_interface
 */

namespace dual_arm_manipulation_planner_interface
{
/** @class DualArmManipulationPlannerInterface
 *  This class defines the interface to the motion planners in OMPL
 */

class DualArmManipulationPlannerInterface
{
public:

  /**
   * @brief Initialize OMPL-based planning for a particular robot model. ROS configuration is read from the specified NodeHandle
   * @param kmodel
   * @param nh
   */
  DualArmManipulationPlannerInterface(const robot_model::RobotModelConstPtr &kmodel,
                                      const ros::NodeHandle &nh = ros::NodeHandle("~"));

  DualArmManipulationPlannerInterface(const robot_model::RobotModelConstPtr& kmodel,
                const planning_interface::PlannerConfigurationMap& pconfig,
                const ros::NodeHandle& nh = ros::NodeHandle("~"));

  virtual ~DualArmManipulationPlannerInterface();

  /**
   * @brief Specify configurations for the planners
   * @param pconfig pconfig Configurations for the different planners
   */
  void setPlannerConfigurations(const planning_interface::PlannerConfigurationMap& pconfig);

  /**
   * @brief Get the configurations for the planners that are already loaded
   * @return pconfig Configurations for the different planners
   */
  const planning_interfacae::PlannerConfigurationMap& getPlannerConfigurations() const
  {
    return context_manager_.getPlannerConfigurations();
  }


protected:

  PlanningContextManager context_manager_;



};
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_DUAL_ARM_MANIPULATION_PLANNER_INTERFACE_H
