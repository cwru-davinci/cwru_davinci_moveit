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

#ifndef CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_GRASPED_OBJECT_HANDLER_H
#define CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_GRASPED_OBJECT_HANDLER_H

#include <ros/ros.h>
#include <string>
#include <moveit_msgs/AttachedCollisionObject.h>

namespace davinci_moveit_object_handling
{

  /** @class
   * @brief Abstract class to serve as a helper for handling attaching and detaching of objects to a robot.
   *
   * TODO: At some stage, this will move to a more general package than this one in MoveIt!. This will
   * happen as soon as another implementation of the Handler is needed, and a general package (maybe
   * in grasp-pkgs repository) is created.
   */

  class DavinciGraspedObjectHandler
  {
  public:
    DavinciGraspedObjectHandler()
    {
    }

    virtual ~DavinciGraspedObjectHandler()
    {
    }

    virtual bool attachObjectToRobot(const std::string &object_name, const std::string &attach_link_name) = 0;

    virtual bool detachObjectFromRobot(const std::string &object_name) = 0;

  };

  /** @class
   * @brief Implementation of GraspedObjectHandler for MoveIt! collision object.
   */

  class DavinciMoveitGraspedObjectHandler : public DavinciGraspedObjectHandler
  {
  public:

    DavinciMoveitGraspedObjectHandler(const ros::NodeHandle &nodeHandle,
                                      const std::vector <std::string> &gripper_link_names,
                                      const std::string &get_planning_scene_service = "/get_planning_scene",
                                      const std::string &set_planning_scene_topic = "/planning_scene");

    virtual ~DavinciMoveitGraspedObjectHandler();

    /**
     * @brief attach object to robot
     * @param object_name
     * @param attach_link_name
     * @return
     */
    virtual bool attachObjectToRobot(const std::string &object_name, const std::string &attach_link_name);

    /**
     * @brief detach object from robot
     * @param object_name
     * @return
     */
    virtual bool detachObjectFromRobot(const std::string &object_name);

    void waitForSubscribers();

  private:

    bool attachObjectToRobot(const std::string &name,
                             const std::string &link_name,
                             const std::vector <std::string> &allowed_touched_links);

    /**
     * @brief check if certain object is already attached on the robot
     * @param name the name of the object
     * @param objs current objects attached on robot
     * @param obj the certain object
     * @return
     */
    static bool hasObject(const std::string &name,
                          const std::vector <moveit_msgs::AttachedCollisionObject> &objs,
                          moveit_msgs::AttachedCollisionObject &obj);

    /**
     * @brief check if certain object is in current collision map
     * @param name the name of the object
     * @param objs current objects in collision map
     * @param obj the certain object
     * @return
     */
    static bool hasObject(const std::string &name,
                          const std::vector <moveit_msgs::CollisionObject> &objs,
                          moveit_msgs::CollisionObject& obj);


    /**
     * @brief remove certain objects from collision objects list
     * @param name the name of the object to remove
     * @param objs collision objects list
     * @return
     */
    static bool removeObject(const std::string& name, std::vector<moveit_msgs::CollisionObject>& objs);

    /**
     * @brief remove certain objects from attached object list
     * @param name the name of the object to remove
     * @param objs attached objects list
     * @return
     */
    static bool removeObject(const std::string& name, std::vector<moveit_msgs::AttachedCollisionObject>& objs);

    void initializePublisher(const std::string& get_planning_scene_service = "/get_planning_scene");

    void initializeClient(const std::string& set_planning_scene_topic = "/planning_scene");

    bool transformPose(const geometry_msgs::Pose &pose, const std::string &from_frame,
                       const std::string &to_frame, geometry_msgs::Pose &p);

    /**
     * @brief Transform all object poses to the given frame
     * @param to_frame
     * @param collision_object
     * @return
     */
    bool transformCollisionObject(const std::string& to_frame, moveit_msgs::CollisionObject& collision_object);

    ros::ServiceClient moveit_planning_scene_diff_client_;
    ros::Publisher moveit_planning_scene_diff_publisher_;

    ros::NodeHandle nh_;

    std::vector<std::string> gripper_link_names_;
  };

typedef boost::shared_ptr<DavinciMoveitGraspedObjectHandler> DavinciMoveitGraspedObjectHandlerPtr;
}


#endif  // CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_GRASPED_OBJECT_HANDLER_H
