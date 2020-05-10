/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Case Western Reserve University
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
 * Description: handoff performance test main function
 */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>

#include <dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <dual_arm_manipulation_planner_interface//hybrid_motion_validator.h>

// Grasp generation and visualization
#include <cwru_davinci_grasp/davinci_simple_needle_grasper.h>
#include <unordered_set>
#include <algorithm>
#include <numeric>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using namespace dual_arm_manipulation_planner_interface;

struct PerformanceStats
{
  double mean;
  double stdev;
  double mid;
  double max;
  double min;
  int failed_num;
  int succeeded_num;

  PerformanceStats()
  {
    mean = 0.0;
    stdev = 0.0;
    mid = 0.0;
    max = 0.0;
    min = 0.0;
    failed_num = 0;
    succeeded_num = 0;
  }
};

void printPerformanceStats
(
const PerformanceStats& pStats,
const std::string& testCase
)
{
  std::cout << "The test case is " << testCase << " and its performance stats are listed below:" << std::endl;
  std::cout << "Number of failed times is " << pStats.failed_num << "\n"
            << "Number of succeeded times is " << pStats.succeeded_num << "\n"
            << "Test case average running time is " << pStats.mean << "s\n"
            << "Test case standard deviation is " << pStats.stdev << "s\n"
            << "Test case max running time is " << pStats.max << "s\n"
            << "Test case min running time is " << pStats.min << "s" << std::endl;
}

class HdfPerformanceTester
{
private:
  std::recursive_mutex m_mutex;

  void getPerformanceStats
  (
  const std::vector<double>& running_time,
  PerformanceStats& stats
  )
  {
    stats.max = *std::max_element(running_time.begin(), running_time.end());
    stats.min = *std::min_element(running_time.begin(), running_time.end());

    double accum = std::accumulate(running_time.begin(), running_time.end(), 0);
    double mean = accum / running_time.size();
    stats.mean = mean;

    std::for_each(running_time.begin(), running_time.end(), [&](const double d)
    {
      accum += (d - mean) * (d - mean);
    });
    stats.stdev = sqrt(accum / (running_time.size() - 1));
  }

  void solveAndPostProcessing
  (
  std::vector<double>& running_time,
  int& failed_num,
  int& succeeded_num,
  const ompl::base::SpaceInformationPtr& pSpaceInfor,
  const ob::ScopedState<HybridObjectStateSpace>& start,
  const ob::ScopedState<HybridObjectStateSpace>& goal
  )
  {
    // std::lock_guard<std::recursive_mutex> locker(m_mutex);

    // create a problem instance
    ompl::base::ProblemDefinitionPtr pPdef(std::make_shared<ob::ProblemDefinition>(pSpaceInfor));
    // set the start and goal states
    pPdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    std::shared_ptr<og::RRTConnect> pPlanner(std::make_shared<og::RRTConnect>(pSpaceInfor));
    // set the problem we are trying to solve for the planner
    pPlanner->setProblemDefinition(pPdef);
    pPlanner->setRange(100.0);
    // perform setup steps for the planner
    pPlanner->setup();
    // print the settings for this space
    pSpaceInfor->printSettings(std::cout);
    // print the problem settings
    pPdef->print(std::cout);
    // attempt to solve the problem within one second of planning time

    auto start_ts = std::chrono::high_resolution_clock::now();
    pSpaceInfor->getStateSpace().get()->as<HybridObjectStateSpace>()->resetTimer();
    ob::PlannerStatus solved = pPlanner->ob::Planner::solve(200.0);
  
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time = finish - start_ts;

    if (solved)
    {
      if (pPdef->hasExactSolution())
      {
        std::cout << "Has exact solution" << std::endl;
        succeeded_num += 1;
        running_time.push_back(planning_time.count());
      }
      else
      {
        std::cout << "Do not have exact solution" << std::endl;
        failed_num += 1;
      }

      og::PathGeometric slnPath = *(pPdef->getSolutionPath()->as<og::PathGeometric>());
      // print the path to screen
      std::cout << "Found solution:\n" << "\n";
      std::cout << "Found solution with " << slnPath.getStateCount()
                << " states and length " << slnPath.length() << std::endl;
      // print the path to screen
      slnPath.printAsMatrix(std::cout);
  
      std::cout << "Writing PlannerData to file’./myPlannerData’" << std::endl;
      ob::PlannerData plannedData(pSpaceInfor);
      pPlanner->getPlannerData(plannedData);
      plannedData.computeEdgeWeights();
  
      std::cout << "Found " << plannedData.numVertices() << " vertices " << "\n";
      std::cout << "Found " << plannedData.numEdges() << " edges " << "\n";
      std::cout << "Actual Planning Time is: " << planning_time.count() << std::endl;
    }
    else
    {
      std::cout << "No solution found" << std::endl;
      failed_num += 1;
    }

    pPlanner->clear();
  }

  void sampleGoalState
  (
  const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
  const ompl::base::SpaceInformationPtr& pSpaceInfor,
  const ob::StateSamplerPtr& pStateSampler,
  const ob::ScopedState<HybridObjectStateSpace>& start,
  const int num_hdf,
  ob::ScopedState<HybridObjectStateSpace>& goal
  )
  {
    pStateSampler->sampleUniform(goal.get());
    int start_arm_index = start->armIndex().value;
    int start_grasp_part = grasp_poses[start->graspIndex().value].part_id;
    int goal_arm_index = goal->armIndex().value;
    int goal_grasp_part = grasp_poses[goal->graspIndex().value].part_id;
    bool same_arm = (start_arm_index == goal_arm_index) ? true : false;
    bool same_grasp_part = (start_grasp_part == goal_grasp_part) ? true : false;

    bool is_gs_valid = pSpaceInfor->isValid(goal.get());

    switch(num_hdf)
    {
      case 1:
        while (same_arm || same_grasp_part || !is_gs_valid)
        {
          pStateSampler->sampleUniform(goal.get());
          is_gs_valid = pSpaceInfor->isValid(goal.get());
          if (!is_gs_valid)
            continue;
          same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
          same_grasp_part = (start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
        }
        return;
      case 2:
        while (!same_arm || same_grasp_part || !is_gs_valid)
        {
          pStateSampler->sampleUniform(goal.get());
          is_gs_valid = pSpaceInfor->isValid(goal.get());
          if (!is_gs_valid)
            continue;
          same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
          same_grasp_part = (start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
        }
        return;
      case 3:
        while (same_arm || !same_grasp_part || !is_gs_valid)
        {
          pStateSampler->sampleUniform(goal.get());
          is_gs_valid = pSpaceInfor->isValid(goal.get());
          if (!is_gs_valid)
            continue;
          same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
          same_grasp_part = (start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
        }
        return;
    }
  }

public:
  // void sampleGoalStateOneHdf
  // (
  // const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
  // const ompl::base::SpaceInformationPtr& pSpaceInfor,
  // const ob::StateSamplerPtr& pStateSampler,
  // const ob::ScopedState<HybridObjectStateSpace>& start,
  // ob::ScopedState<HybridObjectStateSpace>& goal
  // )
  // {
  //   pStateSampler->sampleUniform(goal.get());
  //   int start_arm_index = start->armIndex().value;
  //   int start_grasp_part = grasp_poses[start->graspIndex().value].part_id;
  //   int goal_arm_index = goal->armIndex().value;
  //   int goal_grasp_part = grasp_poses[goal->graspIndex().value].part_id;
  //   bool same_arm = (start_arm_index == goal_arm_index) ? true : false;
  //   bool same_grasp_part = (start_grasp_part == goal_grasp_part) ? true : false;
  
  //   bool is_gs_valid = pSpaceInfor->isValid(goal.get());
  //   while (same_arm || same_grasp_part || !is_gs_valid)
  //   {
  //     pStateSampler->sampleUniform(goal.get());
  //     is_gs_valid = pSpaceInfor->isValid(goal.get());
  //     if (!is_gs_valid)
  //       continue;
  //     same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
  //     same_grasp_part = (start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
  //   }
  // }

  // void sampleGoalStateTwoHdf
  // (
  // const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
  // const ompl::base::SpaceInformationPtr& pSpaceInfor,
  // const ob::StateSamplerPtr& pStateSampler,
  // const ob::ScopedState<HybridObjectStateSpace>& start,
  // ob::ScopedState<HybridObjectStateSpace>& goal
  // )
  // {
  //   pStateSampler->sampleUniform(goal.get());
  //   int start_arm_index = start->armIndex().value;
  //   int start_grasp_part = grasp_poses[start->graspIndex().value].part_id;
  //   int goal_arm_index = goal->armIndex().value;
  //   int goal_grasp_part = grasp_poses[goal->graspIndex().value].part_id;
  //   bool same_arm = (start_arm_index == goal_arm_index) ? true : false;
  //   bool same_grasp_part = (start_grasp_part == goal_grasp_part) ? true : false;

  //   bool is_gs_valid = pSpaceInfor->isValid(goal.get());
  //   while (!same_arm || same_grasp_part || !is_gs_valid)
  //   {
  //     pStateSampler->sampleUniform(goal.get());
  //     is_gs_valid = pSpaceInfor->isValid(goal.get());
  //     if (!is_gs_valid)
  //       continue;
  //     same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
  //     same_grasp_part = (start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
  //   }
  // }

  // void sampleGoalStateThreeHdf
  // (
  // const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
  // const ompl::base::SpaceInformationPtr& pSpaceInfor,
  // const ob::StateSamplerPtr& pStateSampler,
  // const ob::ScopedState<HybridObjectStateSpace>& start,
  // ob::ScopedState<HybridObjectStateSpace>& goal
  // )
  // {
  //   pStateSampler->sampleUniform(goal.get());
  //   int start_arm_index = start->armIndex().value;
  //   int start_grasp_part = grasp_poses[start->graspIndex().value].part_id;
  //   int goal_arm_index = goal->armIndex().value;
  //   int goal_grasp_part = grasp_poses[goal->graspIndex().value].part_id;
  //   bool same_arm = (start_arm_index == goal_arm_index) ? true : false;
  //   bool same_grasp_part = (start_grasp_part == goal_grasp_part) ? true : false;

  //   bool is_gs_valid = pSpaceInfor->isValid(goal.get());
  //   while (same_arm || !same_grasp_part || !is_gs_valid)
  //   {
  //     pStateSampler->sampleUniform(goal.get());
  //     is_gs_valid = pSpaceInfor->isValid(goal.get());
  //     if (!is_gs_valid)
  //       continue;
  //     same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
  //     same_grasp_part = (start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
  //   }
  // }

  PerformanceStats handoffTest
  (
  const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
  int num_test,
  const std::string& objectName,
  const robot_model_loader::RobotModelLoaderConstPtr& pRobotModelLoader,
  const int num_hdf
  )
  {
    // create an instance of state space  // create an instance of state space
    auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size() - 1, grasp_poses));

    // construct an instance of  space information from this state space
    ompl::base::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(hystsp));

    ompl::base::RealVectorBounds se3_xyz_bounds(3);
    se3_xyz_bounds.setLow(0, -0.1);
    se3_xyz_bounds.setHigh(0, 0.1);
    se3_xyz_bounds.setLow(1, -0.2);
    se3_xyz_bounds.setHigh(1, 0.2);
    se3_xyz_bounds.setLow(2, 0.0);
    se3_xyz_bounds.setHigh(2, 0.18);

    hystsp->setSE3Bounds(se3_xyz_bounds);

    si->setStateValidityChecker(
    std::make_shared<HybridStateValidityChecker>(si, pRobotModelLoader->getModel(), objectName));
    si->setMotionValidator(
    std::make_shared<HybridMotionValidator>(si, pRobotModelLoader->getModel(), objectName));
    si->setup();

    ob::StateSamplerPtr stateSampler;  // setup a sampler
    stateSampler = si->allocStateSampler();  // assign HybridStateSampler to stateSampler

    int failed_num = 0;
    int succeeded_num = 0;
    std::vector<double> running_time;
    std::vector<std::thread> threads;

    for (std::size_t i = 0; i < num_test; ++i)
    {
      // create a random start state
      ob::ScopedState<HybridObjectStateSpace> start(hystsp);
      ob::ScopedState<HybridObjectStateSpace> goal(hystsp);

      bool is_ss_valid = false;
      while (!is_ss_valid)
      {
        stateSampler->sampleUniform(start.get());
        is_ss_valid = si->isValid(start.get());
      }

      sampleGoalState(grasp_poses, si, stateSampler, start, num_hdf, goal);

  //    hystsp->printState(start.get(), std::cout);
  //    std::cout << "Start state selected Grasp's part is " << grasp_poses[start->graspIndex().value].part_id << std::endl;
  //    hystsp->printState(goal.get(), std::cout);
  //    std::cout << "Goal state selected Grasp's part is " << grasp_poses[goal->graspIndex().value].part_id << std::endl;
      solveAndPostProcessing(running_time, failed_num, succeeded_num, si, start, goal);
    //   threads.push_back
    //   (
    //     std::thread
    //     (
    //       &HdfPerformanceTester::solveAndPostProcessing,
    //       this,
    //       std::ref(running_time),
    //       std::ref(failed_num),
    //       std::ref(succeeded_num),
    //       std::ref(si),
    //       std::cref(start),
    //       std::cref(goal)
    //     )
    //   );
    }

    // for (std::thread &t : threads)
    // {
    //  if (t.joinable())
    //  {
    //    t.join();
    //  }
    // }

    PerformanceStats stats;
    stats.failed_num = failed_num;
    stats.succeeded_num = succeeded_num;
    getPerformanceStats(running_time, stats);
    return stats;
  }

  // PerformanceStats handoffTest
  // (
  // const ros::NodeHandle& node_handle,
  // const ros::NodeHandle& node_handle_priv,
  // const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
  // int num_test,
  // const std::string& objectName,
  // const robot_model_loader::RobotModelLoaderConstPtr& pRobotModelLoader,
  // void (HdfPerformanceTester::*goalSamplingFcn)(const std::vector<cwru_davinci_grasp::GraspInfo>&,
  //                                               const ompl::base::SpaceInformationPtr&,
  //                                               const ob::StateSamplerPtr&,
  //                                               const ob::ScopedState<HybridObjectStateSpace>&,
  //                                               ob::ScopedState<HybridObjectStateSpace>&)
  // )
  // {
  //   // create an instance of state space  // create an instance of state space
  //   auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size() - 1, grasp_poses));

  //   // construct an instance of  space information from this state space
  //   ompl::base::SpaceInformationPtr si(std::make_shared<ob::SpaceInformation>(hystsp));

  //   ompl::base::RealVectorBounds se3_xyz_bounds(3);
  //   se3_xyz_bounds.setLow(0, -0.1);
  //   se3_xyz_bounds.setHigh(0, 0.1);
  //   se3_xyz_bounds.setLow(1, -0.2);
  //   se3_xyz_bounds.setHigh(1, 0.2);
  //   se3_xyz_bounds.setLow(2, 0.0);
  //   se3_xyz_bounds.setHigh(2, 0.18);

  //   hystsp->setSE3Bounds(se3_xyz_bounds);

  //   si->setStateValidityChecker(
  //   std::make_shared<HybridStateValidityChecker>(si, pRobotModelLoader->getModel(), objectName));
  //   si->setMotionValidator(
  //   std::make_shared<HybridMotionValidator>(si, pRobotModelLoader->getModel(), objectName));
  //   si->setup();

  //   ob::StateSamplerPtr stateSampler;  // setup a sampler
  //   stateSampler = si->allocStateSampler();  // assign HybridStateSampler to stateSampler

  //   int failed_num = 0;
  //   int succeeded_num = 0;
  //   std::vector<double> running_time;
  //   std::vector<std::thread> threads;

  //   for (std::size_t i = 0; i < num_test; ++i)
  //   {
  //     // create a random start state
  //     ob::ScopedState<HybridObjectStateSpace> start(hystsp);
  //     ob::ScopedState<HybridObjectStateSpace> goal(hystsp);

  //     bool is_ss_valid = false;
  //     while (!is_ss_valid)
  //     {
  //       stateSampler->sampleUniform(start.get());
  //       is_ss_valid = si->isValid(start.get());
  //     }

  //     this->*goalSamplingFcn(grasp_poses, si, stateSampler, start, goal);

  // //    hystsp->printState(start.get(), std::cout);
  // //    std::cout << "Start state selected Grasp's part is " << grasp_poses[start->graspIndex().value].part_id << std::endl;
  // //    hystsp->printState(goal.get(), std::cout);
  // //    std::cout << "Goal state selected Grasp's part is " << grasp_poses[goal->graspIndex().value].part_id << std::endl;

  //     // create a problem instance
  //     ompl::base::ProblemDefinitionPtr pdef(std::make_shared<ob::ProblemDefinition>(si));
  
  //     // set the start and goal states
  //     pdef->setStartAndGoalStates(start, goal);

  //     // create a planner for the defined space
  //     std::shared_ptr<og::RRTConnect> planner(std::make_shared<og::RRTConnect>(si));
  //     // set the problem we are trying to solve for the planner
  //     planner->setProblemDefinition(pdef);
  //     planner->setRange(100.0);
  //     // perform setup steps for the planner
  //     planner->setup();
  //     // print the settings for this space
  //     si->printSettings(std::cout);
  //     // print the problem settings
  //     pdef->print(std::cout);
  //     // attempt to solve the problem within one second of planning time

  //     threads.push_back
  //     (
  //       std::thread
  //       (
  //         solveAndPostProcessing,
  //         std::ref(running_time),
  //         std::ref(failed_num),
  //         std::ref(succeeded_num),
  //         std::cref(si),
  //         std::cref(planner),
  //         std::cref(pdef)
  //       )
  //     );
  //   }

  //   for (std::thread &t : threads)
  //   {
  //    if (t.joinable())
  //    {
  //      t.join();
  //    }
  //   }

  //   PerformanceStats stats;
  //   stats.failed_num = failed_num;
  //   stats.succeeded_num = succeeded_num;
  //   getPerformanceStats(running_time, stats);
  //   return stats;
  // }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "handoff_performance_test_main");

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");
  ros::Duration(3.0).sleep();

  cwru_davinci_grasp::DavinciNeedleGrasperBasePtr simpleGrasp =
  boost::make_shared<cwru_davinci_grasp::DavinciNeedleGrasperBase>(
  node_handle_priv, "psm_one", "psm_one_gripper");

  int test_num = 0;
  node_handle_priv.getParam("test_num", test_num);

  std::vector<cwru_davinci_grasp::GraspInfo> grasp_poses = simpleGrasp->getAllPossibleNeedleGrasps(false);

  std::string objectName = "needle_r";
  const robot_model_loader::RobotModelLoaderConstPtr pRobotModelLoader(new robot_model_loader::RobotModelLoader("robot_description"));
  HdfPerformanceTester oneHdfPerformaceTest;
  HdfPerformanceTester twoHdfPerformaceTest;
  HdfPerformanceTester threeHdfPerformaceTest;
  PerformanceStats oneHfStats = oneHdfPerformaceTest.handoffTest(grasp_poses, test_num, objectName, pRobotModelLoader, 1);
  PerformanceStats twoHfStats = twoHdfPerformaceTest.handoffTest(grasp_poses, test_num, objectName, pRobotModelLoader, 2);
  PerformanceStats threeHfStats = threeHdfPerformaceTest.handoffTest(grasp_poses, test_num, objectName, pRobotModelLoader, 3);

  printPerformanceStats(oneHfStats, "One Handoff");
  printPerformanceStats(twoHfStats, "Two Handoff");
  printPerformanceStats(threeHfStats, "Three Handoff");

  return 0;
}
