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

#include <cwru_davinci_moveit_object_handling/davinci_moveit_helper.h>
#include <geometric_shapes/solid_primitive_dims.h>

namespace davinci_moveit_object_handling
{
//  std::ostream& operator<<(std::ostream& o, const Eigen::Quaterniond& q)
//  {
//    o << q.x() << "," << q.y() << "," << q.z() << "," << q.w();
//    return o;
//  }

//  shape_msgs::SolidPrimitive DavinciMoveitHelper::getCone(const double& height, const double& radius)
//  {
//    shape_msgs::SolidPrimitive bv;
//    bv.type = shape_msgs::SolidPrimitive::CONE;
//    bv.dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CONE>::value);
//    bv.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT] = height;
//    bv.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS] = radius;
//    return bv;
//  }

  DavinciMoveitHelper::DavinciMoveitHelper();
  DavinciMoveitHelper::~DavinciMoveitHelper();

  moveit_msgs::Constraints DavinciMoveitHelper::getPoseConstraint(const std::string &link_name,
                                                                  const geometry_msgs::PoseStamped &pose,
                                                                  double tolerance_pos,
                                                                  double tolerance_angle,
                                                                  int type)
  {
    moveit_msgs::Constraints goal;

    if(type <= 1)
    {
      goal.position_constraints.reserve(1);
      moveit_msgs::PositionConstraint &pcm = goal.position_constraints[0];
      pcm.link_name = link_name;
      pcm.target_point_offset.x = 0;
      pcm.target_point_offset.y = 0;
      pcm.target_point_offset.z = 0;

      pcm.constraint_region.primitives.resize(1);
      shape_msgs::SolidPrimitive &bv = pcm.constraint_region.primitives[0];

      bv = getSphere(tolerance_pos);
      pcm.header = pose.header;
      pcm.constraint_region.primitive_poses.resize(1);
      pcm.constraint_region.primitive_poses[0].position = pose.pose.position;

      // orientation of constraint region does not affect anything, since it is a sphere
      pcm.constraint_region.primitive_poses[0].orientation.x = 0.0;
      pcm.constraint_region.primitive_poses[0].orientation.y = 0.0;
      pcm.constraint_region.primitive_poses[0].orientation.z = 0.0;
      pcm.constraint_region.primitive_poses[0].orientation.w = 1.0;
      pcm.weight = 1.0;

      if((type == 1) || (type == 2))
      {
        goal.orientation_constraints.resize(1);
        moveit_msgs::OrientationConstraint &ocm = goal.orientation_constraints[0];
        ocm.link_name = link_name;
        ocm.header = pose.header;
        ocm.orientation = pose.pose.orientation;
        ocm.absolute_x_axis_tolerance = tolerance_angle;
        ocm.absolute_y_axis_tolerance = tolerance_angle;
        ocm.absolute_z_axis_tolerance = tolerance_angle;
        ocm.weight = 1.0;
      }

      return goal;
    }
  }

}


