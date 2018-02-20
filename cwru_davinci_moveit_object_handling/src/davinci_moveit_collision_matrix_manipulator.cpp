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

#include <cwru_davinci_moveit_object_handling/davinci_moveit_collision_matrix_manipulator.h>
#include <cwru_davinci_moveit_object_handling/davinci_moveit_helper.h>

#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/GetCartesianPath.h>

#include <Eigen/Core>

#define DEFAULT_SET_PLANNING_SCENE_TOPIC "/planning_scene"
#define DEFAULT_GET_PLANNING_SCENE_TOPIC "/get_planning_scene"

namespace davinci_moveit_object_handling
{
  DavinciMoveitCollisionMatrixManipulator::DavinciMoveitCollisionMatrixManipulator(const ros::NodeHandle &nodeHandle)
    : nh_(nodeHandle)
  {
    ros::NodeHandle _node("/moveit_object_handling");

    GET_PLANNING_SCENE_TOPIC = DEFAULT_GET_PLANNING_SCENE_TOPIC;
    _node.param<std::string>("moveit_get_planning_scene_topic", GET_PLANNING_SCENE_TOPIC, GET_PLANNING_SCENE_TOPIC);
    ROS_INFO("Got moveit_get_planning_scene_topic: <%s>", GET_PLANNING_SCENE_TOPIC.c_str());


    SET_PLANNING_SCENE_TOPIC = DEFAULT_SET_PLANNING_SCENE_TOPIC;
    _node.param<std::string>("moveit_set_planning_scene_topic", SET_PLANNING_SCENE_TOPIC, SET_PLANNING_SCENE_TOPIC);
    ROS_INFO("Got moveit_set_planning_scene_topic: <%s>", SET_PLANNING_SCENE_TOPIC.c_str());

    moveit_planning_scene_diff_publisher_= nh_.advertise<moveit_msgs::PlanningScene>(SET_PLANNING_SCENE_TOPIC, 1);
    moveit_planning_scene_diff_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>(
      GET_PLANNING_SCENE_TOPIC);

    // only done to ensure connections are made with the planning scene
    // topic/service, this works better together with MoveIt! launch files
    ros::Duration(0.2).sleep();
  }



  DavinciMoveitCollisionMatrixManipulator::~DavinciMoveitCollisionMatrixManipulator()
  {

  }

  bool DavinciMoveitCollisionMatrixManipulator::getCurrentMoveitAllowedCollisionMatrix(
    moveit_msgs::AllowedCollisionMatrix &acm)
  {
    bool is_empty = false;
    moveit_msgs::GetPlanningScene p_scene_srv;

    if (!moveit_planning_scene_diff_client_.call(p_scene_srv))
    {
      ROS_ERROR("Can't obtain planning scene");
      return is_empty;
    }

    acm = p_scene_srv.response.scene.allowed_collision_matrix;
    if(acm.entry_names.empty())
    {
      ROS_ERROR("Collision matrix should not be empty");
      return is_empty;
    }

    //ROS_INFO_STREAM("Matrix: "<<matrix);
    is_empty = true;
    return is_empty;
  }

  bool DavinciMoveitCollisionMatrixManipulator::setAllowedMoveItCollisionMatrix(
    const moveit_msgs::AllowedCollisionMatrix &acm)
  {
    moveit_msgs::PlanningScene planning_scene;

    if(moveit_planning_scene_diff_publisher_.getNumSubscribers() < 1)
    {
      ROS_ERROR("Setting collision matrix won't have any effect!");
      return false;
    }
    planning_scene.is_diff = true;
    planning_scene.allowed_collision_matrix = acm;
    moveit_planning_scene_diff_publisher_.publish(planning_scene);
    return true;
  }


  bool DavinciMoveitCollisionMatrixManipulator::addAllowedMoveitCollision(const std::string &name,
                                                                          const std::vector<std::string> &link_names)
  {
    bool is_added = false;
    moveit_msgs::AllowedCollisionMatrix acm;
    if(!getCurrentMoveitAllowedCollisionMatrix(acm))
    {
      return is_added;
    }

    std::vector<std::string>::iterator object_entry_itr = ensureExistsInACM(name, acm, false);

    int object_idx = object_entry_itr - acm.entry_names.begin();

    std::vector<std::string>::const_iterator itr;
    for(itr = link_names.begin(); itr != link_names.end(); ++itr)
    {
      std::vector<std::string>::iterator link_entry = ensureExistsInACM(*itr, acm, false);

      int link_idx = link_entry - acm.entry_names.begin();
      acm.entry_values[link_idx].enabled[object_idx] = true;
      acm.entry_values[object_idx].enabled[link_idx] = true;
    }

    setAllowedMoveItCollisionMatrix(acm);
    is_added = true;
    return is_added;
  }

  bool DavinciMoveitCollisionMatrixManipulator::setAllowedMoveitCollision(const std::string &name1,
                                                                          const std::string &name2,
                                                                          const bool flag)
  {
    moveit_msgs::AllowedCollisionMatrix acm;
    if(!getCurrentMoveitAllowedCollisionMatrix(acm))
    {
      return false;
    }

    std::vector<std::string>::iterator name1_entry = ensureExistsInACM(name1, acm, false);
    std::vector<std::string>::iterator name2_entry = ensureExistsInACM(name1, acm, false);

    int name1_entry_idx = name1_entry - acm.entry_names.begin();
    int name2_entry_idx = name2_entry - acm.entry_names.begin();

    acm.entry_values[name2_entry_idx].enabled[name1_entry_idx] = flag;
    acm.entry_values[name1_entry_idx].enabled[name2_entry_idx] = flag;

    setAllowedMoveItCollisionMatrix(acm);
    return true;

  }

  void DavinciMoveitCollisionMatrixManipulator::expandMoveitCollisionMatrix(const std::string &name,
                                                                            moveit_msgs::AllowedCollisionMatrix &acm,
                                                                            const bool default_value)
  {
    for (int i = 0; i < acm.entry_names.size(); i++)
    {
      acm.entry_values[i].enabled.push_back(default_value);
    }

    acm.entry_names.push_back(name);

    moveit_msgs::AllowedCollisionEntry ace;
    ace.enabled.assign(acm.entry_names.size(), default_value);
    acm.entry_values.push_back(ace);
  }

  std::vector<std::string>::iterator DavinciMoveitCollisionMatrixManipulator::ensureExistsInACM(
    const std::string& name, moveit_msgs::AllowedCollisionMatrix& acm, const bool init_flag)
  {
    std::vector<std::string>::iterator name_entry_itr = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
    if(name_entry_itr == acm.entry_names.end())
    {
      ROS_DEBUG_STREAM("Could not find object " << name
                                                << " in collision matrix. Inserting.");

      // re-assign the 'name_entry' iterator to the new entry_names place
      expandMoveitCollisionMatrix(name, acm, init_flag);
      name_entry_itr = std::find(acm.entry_names.begin(), acm.entry_names.end(), name);
      if(name_entry_itr == acm.entry_names.end())
      {
        ROS_ERROR("consistency, name should now be in map");
      }
    }

    return name_entry_itr;
  }
}
