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

#ifndef CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_OBJECT_MESSAGE_GENERATOR_H
#define CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_OBJECT_MESSAGE_GENERATOR_H

#include <ros/ros.h>

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs/ObjectInfoRequest.h>
#include <object_msgs/ObjectInfoResponse.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/thread/mutex.hpp>

#include <cwru_davinci_moveit_object_handling/davinci_moveit_collision_matrix_manipulator.h>

namespace davinci_moveit_object_handling
{
/**
 * Takes objects of type object_msgs/Object published a topic and transforms and re-publishes it
 * into a moveit_msgs/CollisionObject.
 *
 * The object information is read from incoming object_msgs/Object messages. If not all information
 * is available, a service of type object_msgs/ObjectInfoRequest.srv is sent, e.g. to get the
 * geometry details when required.
 *
 * There are currently two ways to send object information to MoveIt!:
 *
 * 1. Publishing the collision object on /collision_objects which is read by move_group.
 * 2. Another approach is described in
 *      [this PR2 tutorial](http://docs.ros.org/api/pr2_moveit_tutorials/html/planning/src/doc/planning_scene_ros_api_tutorial.html),
 *      in section *"Add object to environment": publising a planning scene diff (message moveit_msgs::PlanningScene)*.
 *
 * The approach to be taken can be set with a ROS parameter is set for it.
 *
 * The parameters for this class can be specified in a YAML file,
 * which needs to be loaded onto the ROS parameter server
 * under **namespace davinci_moveit_object_handling**. An example is given in the directory *config*, filename "CollisionObjectsGenerator.yaml"
 */

  class DavinciObjectMessageGenerator
  {
  private:

    typedef object_msgs::Object ObjectMsg;
    typedef object_msgs::ObjectInfo ObjectInfoMsg;
    typedef std::map<std::string, moveit_msgs::CollisionObject> ObjToPublishMap;

  public:

    DavinciObjectMessageGenerator(const ros::NodeHandle &node_priv, const ros::NodeHandle &node);

    ~DavinciObjectMessageGenerator();

    bool isConnected() const;

  private:

    void connectPub(const ros::SingleSubscriberPublisher& publisher);

    void disconnectPub(const ros::SingleSubscriberPublisher& publisher);

    void publishCollisionsEvent(const ros::TimerEvent& e);

    /**
      * @brief callback for object message
      */
    void receiveObjectMsgCallback(const ObjectMsg& msg);

    moveit_msgs::CollisionObject getCollisionGeometry(const std::string& name);

    /**
     * @brief updates pose data in obj with information in newObj, leaves all other fields untouched
     */
    void updatePose(const ObjectMsg& newObj, moveit_msgs::CollisionObject& obj);

    /**
     * @brief Helper to transfer contents from ObjectMsg to moveit_msgs::CollisionObject
     */
    moveit_msgs::CollisionObject transferContent(const ObjectMsg& msg, bool skip_geometry);

    std::vector<moveit_msgs::CollisionObject> getCurrentCollisionObjects(bool only_names = true);
    std::set<std::string> getCurrentCollisionObjectNames();

    std::vector<moveit_msgs::AttachedCollisionObject> getCurrentAttachedCollisionObjects();
    std::set<std::string> getCurrentAttachedCollisionObjectNames();

    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;

    std::string OBJECTS_TOPIC;
    std::string REQUEST_OBJECTS_TOPIC;
    std::string COLLISION_OBJECT_TOPIC;
    std::string GET_PLANNING_SCENE_SERVICE;
    std::string SET_PLANNING_SCENE_TOPIC;
    float PUBLISH_COLLISION_RATE_;
    bool USE_PLANNING_SCENE_DIFF_;

    ros::Publisher collision_publisher_;
    ros::Publisher planning_scene_publisher_;

    ros::Subscriber object_subscriber_;

    ros::ServiceClient object_info_client_;
    ros::ServiceClient planning_scene_client_;

    // all objects which were already added
    std::set<std::string> added_objects_;

    // objects to skip as collision objects:
    // whenever an Object.msg message arrives to
    // add this object name, skip it.
    std::set<std::string> skip_objects_;

    // links of the robot to always be allowed to
    // collide with *any* new collision object arriving.
    // Technically can be used for any objects (not only
    // robot links), but it's intended to disable collisions
    // with parts of the robot only.
    std::vector<std::string> allowed_collision_links_;

    DavinciMoveitCollisionMatrixManipulator acmManip_;

    ObjToPublishMap objects_to_publish_;

    boost::mutex mutex_; //mutex for addedObjects and objsToPublish

    ros::Timer publishCollisionsTimer_;

    //is set to false until addedObjects is initialized
    bool init_existing_objects_;

  };
}


#endif  // CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_OBJECT_MESSAGE_GENERATOR_H
