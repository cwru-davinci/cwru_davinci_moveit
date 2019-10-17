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

/* Author: Su Lu <sxl924@case.edu> */

#ifndef CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_THREADSAFE_STATE_STORAGE_H
#define CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_THREADSAFE_STATE_STORAGE_H
#include <moveit/robot_state/robot_state.h>
#include <boost/thread.hpp>

namespace dual_arm_manipulation_planner_interface
{

class TSStateStorage
{
public:

  TSStateStorage(const robot_model::RobotModelConstPtr &kmodel);
  TSStateStorage(const robot_state::RobotState &start_state);
  ~TSStateStorage();

  robot_state::RobotState* getStateStorage() const;

private:

  robot_state::RobotState                                       start_state_;
  mutable std::map<boost::thread::id, robot_state::RobotState*> thread_states_;
  mutable boost::mutex                                                  lock_;
};

typedef boost::shared_ptr<TSStateStorage> TSStateStoragePtr;
}

#endif //CWRU_DAVINCI_DUAL_ARM_MANIPULATION_PLANNER_THREADSAFE_STATE_STORAGE_H
