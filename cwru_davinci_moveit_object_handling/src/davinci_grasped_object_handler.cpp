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

/* Author: Su Lu */

#include <cwru_davinci_moveit_object_handling/davinci_grasped_object_handler.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <convenience_ros_functions/ROSFunctions.h>

namespace davinci_moveit_object_handling
{
  DavinciMoveitGraspedObjectHandler::DavinciMoveitGraspedObjectHandler(const ros::NodeHandle &nodeHandle,
                                                                       const std::vector<std::string> &gripper_link_names,
                                                                       const std::string &get_planning_scene_service,
                                                                       const std::string &set_planning_scene_topic) :
    nh_(nodeHandle), gripper_link_names_(gripper_link_names)
  {
    convenience_ros_functions::ROSFunctions::initSingleton();
    initializePublisher(set_planning_scene_topic);
    initializeClient(get_planning_scene_service);
  }

  DavinciMoveitGraspedObjectHandler::~DavinciMoveitGraspedObjectHandler()
  {
    convenience_ros_functions::ROSFunctions::destroySingleton();
  }

  bool DavinciMoveitGraspedObjectHandler::attachObjectToRobot(const std::string &object_name,
                                                              const std::string &attach_link_name)
  {
    bool is_attached = false;

    if(!attachObjectToRobot(object_name, attach_link_name, gripper_link_names_))
    {
      ROS_ERROR("Could not attach object to robot");
    }
    else
    {
      ROS_INFO_STREAM("Have attached object " << object_name << " to " << attach_link_name);
      is_attached = true;
    }
    return is_attached;
  }

  bool DavinciMoveitGraspedObjectHandler::detachObjectFromRobot(const std::string &object_name)
  {
    bool is_detached = false;
    if(moveit_planning_scene_diff_publisher_.getNumSubscribers() < 1)
    {
      ROS_WARN("detachObjectToRobot: No node subscribed to planning scene publisher.");
      return is_detached;
    }

    moveit_msgs::GetPlanningScene pscene_srv;
    pscene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

    if(!moveit_planning_scene_diff_client_.call(pscene_srv))
    {
      ROS_ERROR("Can't obtain planning scene");
      return is_detached;
    }
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;  // always set to be true either in attach or detach case

    ROS_INFO("Now detaching object '%s' from robot", object_name.c_str());

    moveit_msgs::AttachedCollisionObject attached_obj;
    if(!hasObject(object_name, pscene_srv.response.scene.robot_state.attached_collision_objects, attached_obj))
    {
      ROS_WARN("DavinciMoveitGraspedObjectHandler: Object %s was not attached to robot, but it was tried to detach it.",
               object_name.c_str());
      is_detached = true;
      return is_detached;
    }
    attached_obj.object.operation = attached_obj.object.REMOVE;
    planning_scene.robot_state.attached_collision_objects.clear();
    planning_scene.robot_state.attached_collision_objects.push_back(attached_obj);
    planning_scene.robot_state.is_diff = true;


    // Add object to the planning scene again
    // INFO: This was taken out, as the object recognition (along with MoveI! collision
    // object generator) should take care of adding the object to the scene again.
    // Transform the object into the link coordinate frame:
    /*moveit_msgs::CollisionObject collision_object=attached_obj.object;
    if (!transformCollisionObject(target_frame, collision_object)) {
        ROS_ERROR("DavinciMoveitGraspedObjectHandler: Could nto transform object to world frame");
        return false;
    }
    //send object as MoveIt collision object:
    moveit_msgs::CollisionObject collision_obj = attached_obj.object;
    collision_obj.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene.world.collision_objects.push_back(collision_obj);*/

    moveit_planning_scene_diff_publisher_.publish(planning_scene);

    //ROS_INFO("Successfully detached object.");
    is_detached = true;
    return is_detached;
  }


  void DavinciMoveitGraspedObjectHandler::waitForSubscribers()
  {
    while (moveit_planning_scene_diff_publisher_.getNumSubscribers() == 0)
    {
      ROS_INFO("Waiting for subscribers...");
      ros::Duration(0.5).sleep();
    }
  }

  bool DavinciMoveitGraspedObjectHandler::attachObjectToRobot(const std::string &name,
                                                              const std::string &link_name,
                                                              const std::vector<std::string> &allowed_touched_links)
  {
    bool is_attached_to_robot = false;

    ROS_INFO("DavinciMoveitGraspedObjectHandler: Attaching %s to %s", name.c_str(), link_name.c_str());

    if(moveit_planning_scene_diff_publisher_.getNumSubscribers() < 1)
    {
      ROS_ERROR(
        "DavinciMoveitGraspedObjectHandler: attachObjectToRobot: No node subscribed to planning scene publisher.");
      return is_attached_to_robot;
    }

    moveit_msgs::GetPlanningScene pscene_srv;
    pscene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES |
                                               moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

    if(!moveit_planning_scene_diff_client_.call(pscene_srv))
    {
      ROS_ERROR("DavinciMoveitGraspedObjectHandler: Can't obtain planning scene in order to attach object.");
      return is_attached_to_robot;
    }

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    moveit_msgs::CollisionObject remove_object;

    if(!hasObject(name, pscene_srv.response.scene.world.collision_objects, remove_object))
    {
      ROS_ERROR(
        "DavinciMoveitGraspedObjectHandler: Object %s was not in the scene, but it was tried to attach it to robot.",
        name.c_str());
      return is_attached_to_robot;
    }
    else
    {
      // remove object from planning scene because now it's attached to the robot.
      remove_object.operation = remove_object.REMOVE;
      planning_scene.world.collision_objects.clear();
      planning_scene.world.collision_objects.push_back(remove_object);
    }

    //  transform the object into the link coordinate frame
    if(!transformCollisionObject(link_name, remove_object))
    {
      ROS_ERROR("DavinciMoveitGraspedObjectHandler: Could not transform object to link frame");
      return is_attached_to_robot;
    }

    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.object = remove_object;
    attached_object.object.header.frame_id = link_name;
    attached_object.link_name = link_name;
    attached_object.touch_links = allowed_touched_links;

    attached_object.object.operation = attached_object.object.ADD;
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.robot_state.is_diff = true;

    moveit_planning_scene_diff_publisher_.publish(planning_scene);

    while(true)
    {
      pscene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

      if(!moveit_planning_scene_diff_client_.call(pscene_srv))
      {
        ROS_ERROR("DavinciMoveitGraspedObjectHandler: Can't obtain planning scene");
        return is_attached_to_robot;
      }

      moveit_msgs::AttachedCollisionObject obj;

      if(hasObject(name, pscene_srv.response.scene.robot_state.attached_collision_objects, obj))
      {
        ROS_INFO("DavinciMoveitGraspedObjectHandler: Scene is updated with attached object.");
        break;
      }
      ROS_INFO("DavinciMoveitGraspedObjectHandler: Waiting for scene update to attach object...");
      ros::Duration(0.5).sleep();
    }

    //ROS_INFO("Successfully attached object.");
    is_attached_to_robot = true;
    return is_attached_to_robot;

  }


  bool DavinciMoveitGraspedObjectHandler::hasObject(const std::string &name,
                                                    const std::vector<moveit_msgs::AttachedCollisionObject> &objs,
                                                    moveit_msgs::AttachedCollisionObject &obj)
  {
    for(int i = 0; i < objs.size(); ++i)
    {
      if(objs[i].object.id == name)
      {
        obj = objs[i];
        return true;
      }
    }
    return false;
  }

  bool DavinciMoveitGraspedObjectHandler::hasObject(const std::string &name,
                                                    const std::vector<moveit_msgs::CollisionObject> &objs,
                                                    moveit_msgs::CollisionObject &obj)
  {
    for(int i = 0; i < objs.size(); ++i)
    {
      if(objs[i].id == name)
      {
        obj = objs[i];
        return true;
      }
    }
    return false;
  }


  bool DavinciMoveitGraspedObjectHandler::removeObject(const std::string &name,
                                                       std::vector<moveit_msgs::CollisionObject> &objs)
  {

    for(int i = 0; i < objs.size(); ++i)
    {
      if(objs[i].id == name)
      {
        objs.erase(objs.begin() + i);
        return true;
      }
    }
    return false;
  }


  bool DavinciMoveitGraspedObjectHandler::removeObject(const std::string &name,
                                                       std::vector<moveit_msgs::AttachedCollisionObject> &objs)
  {
    for(int i = 0; i < objs.size(); ++i)
    {
      if(objs[i].object.id == name)
      {
        objs.erase(objs.begin() + i);
        return true;
      }
    }
    return false;
  }


  void DavinciMoveitGraspedObjectHandler::initializePublisher(const std::string &set_planning_scene_topic)
  {
    moveit_planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>(set_planning_scene_topic, 1);
  }

  void DavinciMoveitGraspedObjectHandler::initializeClient(const std::string &get_planning_scene_service)
  {
    moveit_planning_scene_diff_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>(get_planning_scene_service);
  }


  bool DavinciMoveitGraspedObjectHandler::transformPose(const geometry_msgs::Pose &pose, const std::string &from_frame,
                                                        const std::string &to_frame, geometry_msgs::Pose &p)
  {
    bool is_transformed = false;
    if(to_frame.empty() || from_frame.empty())
    {
      ROS_ERROR("DavinciMoveitGraspedObjectHandler::transformPose(): Both frames must be set.");
      return is_transformed;
    }

    if(from_frame == to_frame)
    {
      p = pose;
      is_transformed = true;
      return is_transformed;
    }

    geometry_msgs::PoseStamped new_pose, result_pose;
    new_pose.pose = pose;
    new_pose.header.frame_id = from_frame;
    new_pose.header.stamp = ros::Time(0); //in order to get the most recent transform

    if(convenience_ros_functions::ROSFunctions::Singleton()->transformPose(new_pose, to_frame, result_pose, 1) != 0)
    {
      ROS_ERROR("DavinciMoveitGraspedObjectHandler: Transform into frame %s failed. Ignoring transform.",
                to_frame.c_str());
      return is_transformed;
    }

    p = result_pose.pose;
    is_transformed = true;
    return is_transformed;
  }

  bool DavinciMoveitGraspedObjectHandler::transformCollisionObject(const std::string &to_frame,
                                                                   moveit_msgs::CollisionObject &collision_object)
  {
    bool is_transformed = false;

    for(int i = 0; i < collision_object.primitive_poses.size(); i++)
    {
      geometry_msgs::Pose &p = collision_object.primitive_poses[i];
      if(!transformPose(p, collision_object.header.frame_id, to_frame, p))
      {
        ROS_ERROR("DavinciMoveitGraspedObjectHandler: Could not transform object to link frame %s", to_frame.c_str());
        return is_transformed;
      }

      if(i < collision_object.mesh_poses.size())
      {
        p = collision_object.mesh_poses[i];
        collision_object.mesh_poses[i] = p;
        if(!transformPose(p, collision_object.header.frame_id, to_frame, p))
        {
          ROS_ERROR("DavinciMoveitGraspedObjectHandler: Could not transform object mesh to link frame %s",
                    to_frame.c_str());
          return is_transformed;
        }
      }

      if(i < collision_object.plane_poses.size())
      {
        p = collision_object.plane_poses[i];
        collision_object.plane_poses[i] = p;
        if(!transformPose(p, collision_object.header.frame_id, to_frame, p))
        {
          ROS_ERROR("GraspObjectHandler: Could not transform object plane to link frame %s", to_frame.c_str());
          return is_transformed;
        }
      }
    }
    collision_object.header.frame_id = to_frame;
    collision_object.header.stamp = ros::Time::now();
    is_transformed = true;
    return is_transformed;
  }
}
