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
 *
 */

#include <moveit/dual_arm_manipulation_planner_interface/state_validity_checker.h>
#include <moveit/ompl_interface/ompl_planning_context.h>
#include <moveit/profiler/profiler.h>
#include <ros/ros.h>


using namespace dual_arm_manipulation_planner_interface;

StateValidityChecker::StateValidityChecker(const std::string& robot_name,
                                           const std::string& group_name,
                                           const std::string& object_name,
                                           const std::vector<cwru_davinci_grasp::GraspInfo>& possible_grasps,
                                           const ompl::base::SpaceInformationPtr &si)
  : ompl::base::StateValidityChecker(si), robot_model_loader_(robot_name), possible_grasps_(possible_grasps)

{
  kmodel_.reset(
    new robot_model::RobotModel(robot_model_loader_.getModel()->getURDF(), robot_model_loader_.getModel()->getSRDF()));

  planning_scene_.reset(new planning_scene::PlanningScene(kmodel_));

  complete_initial_robot_state_.reset(new robot_state::RobotState(kmodel_));

  tss_(complete_initial_robot_state_);

  collision_request_simple_.group_name = group_name;

  collision_request_with_cost_.group_name = group_name;

  collision_request_simple_verbose_ = collision_request_simple_;
  collision_request_simple_verbose_.verbose = true;
}

void StateValidityChecker::setVerbose(bool flag)
{
  verbose_ = flag;
}

double StateValidityChecker::cost(const ompl::base::State* state) const
{

}


double StateValidityChecker::clearance(const ompl::base::State* state) const
{

}

bool StateValidityChecker::isValidWithoutCache(const ompl::base::State* state, bool verbose) const
{

}

bool StateValidityChecker::isValidWithCache(const ompl::base::State* state, bool verbose) const
{
  robot_state::RobotState* kstate = tss_.getStateStorage();


}

bool StateValidityChecker::convertObjectToRobotState(const ompl::base::State* state, robot_state::RobotState* robot_state)
{

}
