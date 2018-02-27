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

#include <cwru_davinci_moveit_object_handling/davinci_object_message_generator.h>
#include <iostream>

#define DEFAULT_COLLISION_OBJECT_TOPIC "collision_object"
#define DEFAULT_OBJECTS_TOPIC "world/objects"
#define DEFAULT_REQUEST_OBJECTS_TOPIC "world/request_object"
#define DEFAULT_GET_PLANNING_SCENE_SERVICE "/get_planning_scene"
#define DEFAULT_SET_PLANNING_SCENE_TOPIC "/planning_scene"
#define DEFAULT_USE_PLANNING_SCENE_DIFF true

#define DEFAULT_PUBLISH_COLLISION_RATE 30

namespace davinci_moveit_object_handling
{
  DavinciObjectMessageGenerator::DavinciObjectMessageGenerator(const ros::NodeHandle &node_priv,
                                                               const ros::NodeHandle &node)
    : private_nh_(node_priv), nh_(node), acmManip_(node_priv)
  {
    ros::NodeHandle _node("/moveit_object_handling");
    _node.param<std::string>("world_objects_topic", OBJECTS_TOPIC, DEFAULT_OBJECTS_TOPIC);
    ROS_INFO("Got objects topic name: <%s>", OBJECTS_TOPIC.c_str());

    _node.param<std::string>("request_object_service", REQUEST_OBJECTS_TOPIC, DEFAULT_REQUEST_OBJECTS_TOPIC);
    ROS_INFO("Got object service topic name: <%s>", REQUEST_OBJECTS_TOPIC.c_str());

    _node.param<std::string>("collision_object_topic", COLLISION_OBJECT_TOPIC, DEFAULT_COLLISION_OBJECT_TOPIC);
    ROS_INFO("Got collision objects topic name: <%s>", COLLISION_OBJECT_TOPIC.c_str());

//    GET_PLANNING_SCENE_SERVICE = DEFAULT_GET_PLANNING_SCENE_SERVICE;
    _node.param<std::string>("moveit_get_planning_scene_topic", GET_PLANNING_SCENE_SERVICE,
                             DEFAULT_GET_PLANNING_SCENE_SERVICE);
    ROS_INFO("Got moveit_get_planning_scene_topic: <%s>", GET_PLANNING_SCENE_SERVICE.c_str());

//    SET_PLANNING_SCENE_TOPIC = DEFAULT_SET_PLANNING_SCENE_TOPIC;
    _node.param<std::string>("moveit_set_planning_scene_topic", SET_PLANNING_SCENE_TOPIC,
                             DEFAULT_SET_PLANNING_SCENE_TOPIC);
    ROS_INFO("Got moveit_set_planning_scene_topic: <%s>", SET_PLANNING_SCENE_TOPIC.c_str());

    _node.param<bool>("use_planning_scene_diff", USE_PLANNING_SCENE_DIFF_, DEFAULT_USE_PLANNING_SCENE_DIFF);
    ROS_INFO("Got use_planning_scene_diff: <%i>", USE_PLANNING_SCENE_DIFF_);

    std::stringstream def_coll_rate;
    def_coll_rate << DEFAULT_PUBLISH_COLLISION_RATE;
    std::string _PUBLISH_COLLISION_RATE = def_coll_rate.str();
    _node.param<std::string>("publish_collision_rate", _PUBLISH_COLLISION_RATE, _PUBLISH_COLLISION_RATE);
    PUBLISH_COLLISION_RATE_ = atof(_PUBLISH_COLLISION_RATE.c_str());

    std::string skip_string;
    _node.param<std::string>("skip_objects", skip_string, "");
    ROS_INFO("Objects to skip: %s", skip_string.c_str());

    char *str = (char *) skip_string.c_str();
    char *pch = strtok(str, " ,;");

    while(pch != NULL)
    {
      //ROS_INFO("%s\n",pch);
      skip_objects_.insert(std::string(pch));
      pch = strtok(NULL, " ,;");
    }

    std::string allowed_coll_string;
    _node.param<std::string>("allowed_collision_links", allowed_coll_string, "");
    ROS_INFO("Objects to allow collide: %s", allowed_coll_string.c_str());
    str = (char *) allowed_coll_string.c_str();
    pch = strtok(str, " ,;");
    while(pch != NULL)
    {
      //ROS_INFO("%s\n",pch);
      allowed_collision_links_.push_back(std::string(pch));
      pch = strtok(NULL, " ,;");
    }

    if(REQUEST_OBJECTS_TOPIC != "")
    {
      object_info_client_ = nh_.serviceClient<object_msgs::ObjectInfo>(REQUEST_OBJECTS_TOPIC);
    }

    planning_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>(GET_PLANNING_SCENE_SERVICE);

    object_subscriber_ = nh_.subscribe(OBJECTS_TOPIC, 100, &DavinciObjectMessageGenerator::receiveObjectMsgCallback,
                                       this);

    ros::SubscriberStatusCallback conn = boost::bind(&DavinciObjectMessageGenerator::connectPub, this, _1);
    ros::SubscriberStatusCallback disconn = boost::bind(&DavinciObjectMessageGenerator::disconnectPub, this, _1);

    if(!USE_PLANNING_SCENE_DIFF_)
    {
      collision_publisher_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_OBJECT_TOPIC, 100, conn, disconn);
    }
    else
    {
      planning_scene_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>(SET_PLANNING_SCENE_TOPIC, 100, conn,
                                                                            disconn);
    }

    ros::Rate rate(PUBLISH_COLLISION_RATE_);
    publishCollisionsTimer_ = private_nh_.createTimer(rate, &DavinciObjectMessageGenerator::publishCollisionsEvent,
                                                      this);

    init_existing_objects_ = false;

  }


  DavinciObjectMessageGenerator::~DavinciObjectMessageGenerator()
  {

  }

  bool DavinciObjectMessageGenerator::isConnected() const
  {
    bool connected = false;
    if(!USE_PLANNING_SCENE_DIFF_)
    {
      connected = (collision_publisher_.getNumSubscribers() > 0);
    }
    else
    {
      connected = (planning_scene_publisher_.getNumSubscribers() > 0);
    }
    return connected;
  }


  void DavinciObjectMessageGenerator::connectPub(const ros::SingleSubscriberPublisher &publisher)
  {
    mutex_.lock();

    // get all collision objects currently in the scene
    added_objects_ = getCurrentCollisionObjectNames();

    // also add the always allowed collision links for each object
    for(std::set<std::string>::iterator itr = added_objects_.begin(); itr != added_objects_.end(); ++itr)
    {
      acmManip_.addAllowedMoveitCollision(*itr, allowed_collision_links_);
    }
    mutex_.unlock();
    init_existing_objects_ = true;
  }


  void DavinciObjectMessageGenerator::disconnectPub(const ros::SingleSubscriberPublisher& publisher)
  {
    if(!isConnected())
    {
      init_existing_objects_ = false;
    }
  }


  void DavinciObjectMessageGenerator::publishCollisionsEvent(const ros::TimerEvent& e)
  {
    if(!isConnected())
    {
      return;
    }

    mutex_.lock();
    ObjToPublishMap::iterator it;

    for(it = objects_to_publish_.begin(); it != objects_to_publish_.end(); ++it)
    {
      if (!USE_PLANNING_SCENE_DIFF_)  // planning scene is same as before
      {
        collision_publisher_.publish(it->second);
      }
      else
      {
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.world.collision_objects.push_back(it->second);
        planning_scene.is_diff = true;
        planning_scene_publisher_.publish(planning_scene);
      }
    }

    objects_to_publish_.clear();
    mutex_.unlock();
  }

  void DavinciObjectMessageGenerator::receiveObjectMsgCallback(const ObjectMsg& msg)
  {
    if (!isConnected())
    {
      // lost connection to subscribers, so require new initialisation
      // ROS_INFO("ObjectMessageGenerator: No subscribers");
      init_existing_objects_ = false;
      return;
    }

    if (!init_existing_objects_)
    {
      ROS_WARN("DavinciObjectMessageGenerator: No initialisation of objects yet.");
      return; //the existing objects haven't been initialised yet
    }

    std::string id = msg.name;
    if (skip_objects_.find(id) != skip_objects_.end())
    {
      // this is an object to be skipped
      return;
    }

    std::set<std::string> current_attached_objs = getCurrentAttachedCollisionObjectNames();
    if(current_attached_objs.find(id) != current_attached_objs.end())
    {
      // This mean: the object is already attached to the robot
      return;
    }


    boost::unique_lock<boost::mutex>(mutex_);

    ObjToPublishMap::iterator existing = objects_to_publish_.find(id);
    if(existing != objects_to_publish_.end())
    {
      // we already have this object to publish, only allows to overwrite the poses
      updatePose(msg, existing->second);
    }

    moveit_msgs::CollisionObject obj;
    if(added_objects_.find(id) == added_objects_.end())
    {
      // this is a new object
      if(msg.content == object_msgs::Object::POSE)
      {
        ROS_INFO("Object not added yet to the system, so retreiving object geometry... %s", __FILE__);
        obj = getCollisionGeometry(id);
        if(obj.id != id)
        {
          ROS_ERROR_STREAM("Object '" << id << "' geometry could not be retrieved. It cannot be properly added to MoveIt!");
          return;
        }
      }
      else
      {
        obj = transferContent(msg, false);
      }
      added_objects_.insert(id);
      acmManip_.addAllowedMoveitCollision(id, allowed_collision_links_);
    }
    else  // We already have had geometry sent for this object.
    {
      // We may want to enforce a MOVE operation.
      // Only if the object is not currently in the MoveIt! collision environment, or
      // if geometry is actually specified in the message (e.g. to change shape), we'll consider another ADD.

      if( (msg.content == object_msgs::Object::SHAPE) && !msg.primitives.empty() )
      {
        obj = transferContent(msg, false);
      }
      else
      {
        // this is either a MOVE operation, or the geometry is empty.
        // if the object is not in the MoveIt! collision environment, we have to request the geometry.
        std::set<std::string> curr_coll_objs = getCurrentCollisionObjectNames();
        bool object_in_scene = curr_coll_objs.find(id) != curr_coll_objs.end();
        moveit_msgs::CollisionObject current_object;
        bool use_current_object_geometry = false;

        // if the object is not in the MoveIt! environment, or this is a SHAPE operation but the
        // geometry was not specified, then retrieve the geometry, as we have to do a MoveIt ADD operation.
        if(!object_in_scene || (msg.content == object_msgs::Object::SHAPE) && msg.primitives.empty())
        {
          ROS_INFO_STREAM("Object '" << id << "'not added yet to the system, so retreiving object geometry...");
          current_object = getCollisionGeometry(id);
          if((current_object.id == id) && (!current_object.primitives.empty()))
          {
            // successfully retrieved geometry.
            use_current_object_geometry = true;
          }
          else
          {
            ROS_ERROR_STREAM("Object '" << id << "' geometry could not be retrieved. It cannot be properly added to MoveIt! again.");
          }
        }

        if((msg.content == object_msgs::Object::SHAPE) && object_in_scene)
        {
          ROS_WARN("We already have had geometry sent for object '%s', so enforcing a MOVE operation.",
                   msg.name.c_str());
        }

        obj = transferContent(msg, true);  // transfer all content except geometry

        if(use_current_object_geometry)
        {
          obj.primitives = current_object.primitives;
          obj.meshes = current_object.meshes;
          obj.mesh_poses = current_object.mesh_poses;
        }

        // if the object is not in the scene, we have to enforce an ADD operation.
        if (!object_in_scene) obj.operation=moveit_msgs::CollisionObject::ADD;
      }
    }

    objects_to_publish_.insert(std::make_pair(id, obj));
  }


  moveit_msgs::CollisionObject DavinciObjectMessageGenerator::getCollisionGeometry(const std::string& name)
  {
    ROS_INFO("Received information for new object %s, adding it by requesting mesh information...", name.c_str());
    object_msgs::ObjectInfo srv;
    srv.request.name = name;
    srv.request.get_geometry = true;
    if(!object_info_client_.call(srv))
    {
      ROS_ERROR("Could not add object %s because service request failed.", name.c_str());
      return moveit_msgs::CollisionObject();
    }
    ROS_INFO("Object added");
    return transferContent(srv.response.object, false);
  }


  void DavinciObjectMessageGenerator::updatePose(const ObjectMsg& newObj, moveit_msgs::CollisionObject& obj)
  {
    if(obj.header.frame_id != newObj.header.frame_id)
    {
      ROS_WARN("messages not specified in same frame! %s %s", obj.header.frame_id.c_str(),
               newObj.header.frame_id.c_str());
    }

    if(obj.id != newObj.name)
    {
      ROS_ERROR("Not referring to same object to update the pose!");
    }

    obj.header = newObj.header;
    obj.primitive_poses = newObj.primitive_poses;
    obj.mesh_poses = newObj.mesh_poses;
  }


  moveit_msgs::CollisionObject DavinciObjectMessageGenerator::transferContent(const ObjectMsg& msg, bool skip_geometry)
  {
    moveit_msgs::CollisionObject obj;

    obj.header = msg.header;
    obj.id = msg.name;

    if(!skip_geometry)
    {
      obj.primitives = msg.primitives;
    }
    obj.primitive_poses = msg.primitive_poses;

    if (!skip_geometry)
    {
      obj.meshes = msg.meshes;
    }
    obj.mesh_poses = msg.mesh_poses;

    if((msg.content == ObjectMsg::POSE) || skip_geometry)
    {
      obj.operation = moveit_msgs::CollisionObject::MOVE;
    }
    else if (msg.content == ObjectMsg::SHAPE)
    {
      obj.operation = moveit_msgs::CollisionObject::ADD;
    }

    return obj;

  }

  std::vector<moveit_msgs::CollisionObject> DavinciObjectMessageGenerator::getCurrentCollisionObjects(bool only_names)
  {
    if(!planning_scene_client_.exists())
    {
      return std::vector<moveit_msgs::CollisionObject>();
    }

    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    if(!only_names)
    {
      srv.request.components.components =
        srv.request.components.components | moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    }

    if(!planning_scene_client_.call(srv))
    {
      ROS_ERROR("Can't obtain planning scene");
      return std::vector<moveit_msgs::CollisionObject>();
    }

    moveit_msgs::PlanningScene& scene = srv.response.scene;
    return scene.world.collision_objects;
  }

  std::set<std::string> DavinciObjectMessageGenerator::getCurrentCollisionObjectNames()
  {
    std::vector<moveit_msgs::CollisionObject> obj = getCurrentCollisionObjects(true);

    std::set<std::string> ret;

    for(std::vector<moveit_msgs::CollisionObject>::iterator itr = obj.begin(); itr != obj.end(); ++itr)
    {
      ret.insert(itr->id);
    }

    return ret;
  }


  std::vector<moveit_msgs::AttachedCollisionObject> DavinciObjectMessageGenerator::getCurrentAttachedCollisionObjects()
  {
    if(!planning_scene_client_.exists())
    {
      return std::vector<moveit_msgs::AttachedCollisionObject>();  // return a empty list
    }

    moveit_msgs::GetPlanningScene p_scene_srv;
    p_scene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

    if(!planning_scene_client_.call(p_scene_srv))
    {
      ROS_ERROR("Can't obtain planning scene");
      return std::vector<moveit_msgs::AttachedCollisionObject>();
    }

    moveit_msgs::PlanningScene &planning_scene = p_scene_srv.response.scene;
    return planning_scene.robot_state.attached_collision_objects;
  }

  std::set<std::string> DavinciObjectMessageGenerator::getCurrentAttachedCollisionObjectNames()
  {
    std::vector<moveit_msgs::AttachedCollisionObject> attached_obj = getCurrentAttachedCollisionObjects();

    std::set<std::string> ret;
    for(std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = attached_obj.begin();
        it != attached_obj.end(); ++it)
    {

      ret.insert(it->object.id);
    }

    return ret;
  }


}