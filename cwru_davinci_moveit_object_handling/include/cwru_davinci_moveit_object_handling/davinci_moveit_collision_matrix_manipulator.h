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

#ifndef CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_MOVEIT_COLLISION_MATRIX_MANIPULATOR_H
#define CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_MOVEIT_COLLISION_MATRIX_MANIPULATOR_H

#include <ros/ros.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/CollisionObject.h>

namespace davinci_moveit_object_handling
{

/**
 * @brief Provides helper functions to manipulate the MoveIt! Allowed Collision Matrix (ACM).
 *
 * The ACM saves the expensive collision checking operation by a matrix of flags, where
 * when the flag is 1, it specifies that a collision check between the two objects is not needed
 * (either the two never collide, or have explicitly been allowed to collide).
 * The collision matrix is a symmetric matrix.
 *
 * The collision matrix is retrieved and set by the ROS MoveIt! planning scene service and topic.
 *
 */

  class DavinciMoveitCollisionMatrixManipulator
  {
  public:
    DavinciMoveitCollisionMatrixManipulator(const ros::NodeHandle &nodeHandle);

    ~DavinciMoveitCollisionMatrixManipulator();

    bool getCurrentMoveitAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix &acm);

    /**
     * @brief sets the given collision matrix in MoveIt!
     * @param acm the matrix to be set in Moveit
     */
    bool setAllowedMoveItCollisionMatrix(const moveit_msgs::AllowedCollisionMatrix& acm);

    /**
     * @brief allow collision btw collision object @param name and all links of the robot given in @param link_names
     * @param name collision allowed object
     * @param link_names collision allowed robot link names list
     * @return
     */
    bool addAllowedMoveitCollision(const std::string &name, const std::vector <std::string> &link_names);

    /**
     * @brief Set the pair <name1, name2> in the collision matrix with the @param flag. "true" is for allowed collisions
     * (no collision checks needed).
     * If entries for name1 and name2 don't exist, they are added to the collision matrix,
     * and by default are not allowed to collide with any of the other entries (set to "false" for
     * all other objects in ACM).
     * @param name1
     * @param name2
     * @param flag
     * @return
     */
    bool setAllowedMoveitCollision(const std::string &name1, const std::string &name2, const bool flag);


//    /**
//     * @brief Attaches the object which is known to MoveIt! by @param name to the link @param link_name
//     * of the robot. The links of the hand which are allowed to touch the object are to
//     * be given in @param allowed_touch_links.
//     * @param name
//     * @param link_name
//     * @param allowedTouchLinks
//     * @return false if MoveIt! planning scene couldn't be reached or if the object is
//     * not known as collision object to MoveIt!.
//     */
//    bool attachMoveitObjectToRobot(const std::string &name, const std::string &link_name,
//                                   const std::vector <std::string> &allowed_touch_links);
//
//    /**
//     * @brief Detaches the object which has previously been attached with attachMoveitObjectToRobot()
//     * from the robot.
//     * @param name
//     * @return false if it wasn't previously attached or MoveIt! planning scene
//     * could not be reached.
//     */
//    bool detachMoveitObjectFromRobot(const std::string &name);

  private:

    /**
     * @brief expands the collision matrix by appending one row, and one column, and set all fields to @param default_val.
     */
    void expandMoveitCollisionMatrix(const std::string &name, moveit_msgs::AllowedCollisionMatrix &acm,
                                     const bool default_value);

    void getMoveitScene(octomap_msgs::OctomapWithPose &octomap,
                        std::vector <moveit_msgs::CollisionObject> &collision_objects);

//    /**
//     * @brief check if certain object is already attached on the robot
//     * @param name the name of the object
//     * @param objs current objects attached on robot
//     * @param obj the certain object
//     * @return
//     */
//    bool hasObject(const std::string &name,
//                   const std::vector <moveit_msgs::AttachedCollisionObject> &objs,
//                   moveit_msgs::AttachedCollisionObject &obj);
//
//    /**
//     * @brief check if certain object is in current collision map
//     * @param name the name of the object
//     * @param objs current objects in collision map
//     * @param obj the certain object
//     * @return
//     */
//    bool hasObject(const std::string &name,
//                   const std::vector <moveit_msgs::CollisionObject> &objs,
//                   moveit_msgs::CollisionObject &obj);
//
//
//    /**
//     * @brief remove certain objects from collision objects list
//     * @param name the name of the object to remove
//     * @param objs collision objects list
//     * @return
//     */
//    bool removeObject(const std::string &name, std::vector <moveit_msgs::CollisionObject> &objs);
//
//    /**
//     * @brief remove certain objects from attached object list
//     * @param name the name of the object to remove
//     * @param objs attached objects list
//     * @return
//     */
//    bool removeObject(const std::string &name, std::vector <moveit_msgs::AttachedCollisionObject> &objs);
//
//    /**
//     * @brief makes sure an entry for this name is added to the collision matrix if it doesn't exist,
//     * and then returns the iterator to the ACM.entry_names for it.
//     * @param name
//     * @param acm
//     * @param init_flag
//     * @return
//     */
    std::vector<std::string>::iterator ensureExistsInACM(
      const std::string& name, moveit_msgs::AllowedCollisionMatrix& acm, const bool init_flag);

    std::string SET_PLANNING_SCENE_TOPIC;
    std::string GET_PLANNING_SCENE_TOPIC;

    ros::NodeHandle nh_;

    ros::ServiceClient moveit_planning_scene_diff_client_;
    ros::Publisher moveit_planning_scene_diff_publisher_;
  };
}

#endif  // CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_MOVEIT_COLLISION_MATRIX_MANIPULATOR_H
