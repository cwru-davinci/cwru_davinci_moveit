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

#ifndef CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_MOVEIT_HELPER_H
#define CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_MOVEIT_HELPER_H

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
#include <shape_msgs/SolidPrimitive.h>

#include <moveit_msgs/Constraints.h>
#include <eigen_conversions/eigen_msg.h>

namespace davinci_moveit_object_handling
{

  class DavinciMoveitHelper
  {
    typedef boost::shared_ptr<moveit_msgs::PositionConstraint> PositionConstraintPtr;
    typedef boost::shared_ptr<shape_msgs::SolidPrimitive> SolidPrimitivePtr;

    DavinciMoveitHelper();
    ~DavinciMoveitHelper();

    /**
     * Adds goal constraints for the link to be at this pose.
     * \param type if 0, only position is considered. If 1, position and
     *      orientation are considered, and if 2 then only orientation is considered.
     */
    static moveit_msgs::Constraints getPoseConstraint(const std::string &link_name,
                                                      const geometry_msgs::PoseStamped &pose,
                                                      double tolerance_pos,
                                                      double tolerance_angle,
                                                      int type);



  };

}

#endif //CWRU_DAVINCI_MOVEIT_OBJECT_HANDLING_DAVINCI_MOVEIT_HELPER_H
