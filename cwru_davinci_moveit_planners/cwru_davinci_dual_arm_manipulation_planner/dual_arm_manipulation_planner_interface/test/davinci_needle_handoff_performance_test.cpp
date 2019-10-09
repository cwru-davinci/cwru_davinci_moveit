//
// Created by sulu on 6/10/19.
//

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>
#include <ompl/util/RandomNumbers.h>

#include <moveit/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h>
#include <moveit/dual_arm_manipulation_planner_interface//hybrid_motion_validator.h>
#include <moveit/dual_arm_manipulation_planner_interface/hybrid_valid_state_sampler.h>
#include <moveit/dual_arm_manipulation_planner_interface/davinci_needle_handoff_execution.h>

//moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
};

void printPerformanceStats(const PerformanceStats& pStats, const std::string& testCase)
{
  std::cout << "The test case is " << testCase << " and its performance stats are listed below:" << std::endl;
  std::cout << "Number of failed times is " << pStats.failed_num << "\n"
            << "Number of succeeded times is " << pStats.succeeded_num << "\n"
            << "Test case average running time is " << pStats.mean << "s\n"
            << "Test case standard deviation is " << pStats.stdev << "s\n"
            << "Test case max running time is " << pStats.max << "s\n"
            << "Test case min running time is " << pStats.min << "s" << std::endl;
}

bool findValidGrasp(const ompl::base::SpaceInformationPtr &si, ob::ScopedState<HybridObjectStateSpace> cstate,
                    const std::vector<cwru_davinci_grasp::GraspInfo> &possible_grasps, int from_part_id = 0,
                    bool pick_right_part = false)
{
  bool is_found = false;
  std::unordered_set<int> invalid_grasp_list;
  ompl::RNG randNumGenerator;
  int random_range = possible_grasps.size() - 1;
  int stop_it_num = possible_grasps.size();

  while (invalid_grasp_list.size() <
         stop_it_num)  // as long as element in invalid_grasp_list <= possible_grasp size do while
  {
    int random_grasp_index = randNumGenerator.uniformInt(0, random_range);
    if (invalid_grasp_list.find(random_grasp_index) == invalid_grasp_list.end())  // element is not in the container
    {
      if (pick_right_part)
      {
        int grasp_part = possible_grasps[random_grasp_index].part_id;
        // first round screen
        if (grasp_part == from_part_id)  // if failed insert to invalid_grasp_list
        {
          invalid_grasp_list.insert(random_grasp_index);
          continue;
        }
      }
      // if past first round scree then check validity here
      cstate->graspIndex().value = random_grasp_index;
      if (si->isValid(cstate.get()))
      {
        is_found = true;
        return is_found;
      }
      invalid_grasp_list.insert(random_grasp_index);
    }
  }
}

void getPerformanceStats(const std::vector<double>& running_time, PerformanceStats& stats)
{
  stats.max = *std::max_element(running_time.begin(), running_time.end());
  stats.min = *std::min_element(running_time.begin(), running_time.end());


  double accum = std::accumulate(running_time.begin(), running_time.end(), 0);
  double mean = accum / running_time.size();
  stats.mean = mean;

  std::for_each (running_time.begin(), running_time.end(), [&](const double d) {
    accum += (d - mean) * (d - mean);
  });
  stats.stdev = sqrt(accum / (running_time.size()-1));
}

// return an sampler
ob::ValidStateSamplerPtr allocHybridValidStateSampler(const ob::SpaceInformation *si)
{

  // we can perform any additional setup / configuration of a sampler here,
  // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
  return std::make_shared<HybridValidStateSampler>("robot_description", si);
}

PerformanceStats oneHandoffTest(const ros::NodeHandle &node_handle,
                            const ros::NodeHandle &node_handle_priv,
                            const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
                            int num_test)
{
  std::string object_name = "needle_r";
  std::string robot_name = "robot_description";
  // create an instance of state space
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size(), grasp_poses));

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ompl::base::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.101);
  se3_xyz_bounds.setHigh(0, 0.101);
  se3_xyz_bounds.setLow(1, -0.06);
  se3_xyz_bounds.setHigh(1, 0.09);
  se3_xyz_bounds.setLow(2, 0.266);
  se3_xyz_bounds.setHigh(2, 0.496);

  hystsp->setSE3Bounds(se3_xyz_bounds);

  si->setStateValidityChecker(
    std::make_shared<HybridStateValidityChecker>(node_handle, robot_name, object_name, si));
  si->setMotionValidator(
    std::make_shared<HybridMotionValidator>(node_handle, node_handle_priv, robot_name, object_name, si));
  si->setup();

  ob::StateSamplerPtr stateSampler;  // setup a sampler
  stateSampler = si->allocStateSampler();  // assign HybridStateSampler to stateSampler

  int failed_num = 0;
  int succeeded_num = 0;
  std::vector<double> running_time;

  for (int i = 0; i < num_test; i++)
  {
    // create a random start state
    ob::ScopedState<HybridObjectStateSpace> start(hystsp);
    ob::ScopedState<HybridObjectStateSpace> goal(hystsp);

    bool is_ss_valid= false;
    while(!is_ss_valid)
    {
      stateSampler->sampleUniform(start.get());
      is_ss_valid = si->isValid(start.get());
    }

    stateSampler->sampleUniform(goal.get());

    int start_arm_index = start->armIndex().value;
    int start_grasp_part = grasp_poses[start->graspIndex().value].part_id;
    int goal_arm_index = goal->armIndex().value;
    int goal_grasp_part = grasp_poses[goal->graspIndex().value].part_id;
    bool same_arm = (start_arm_index == goal_arm_index) ? true : false;
    bool same_grasp_part = (start_grasp_part == goal_grasp_part) ? true : false;

    bool is_gs_valid = si->isValid(goal.get());
    while(same_arm || same_grasp_part || !is_gs_valid)
    {
      stateSampler->sampleUniform(goal.get());
      is_gs_valid = si->isValid(goal.get());
      if(!is_gs_valid)
        continue;
      same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
      same_grasp_part = ( start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
    }

//    hystsp->printState(start.get(), std::cout);
//    std::cout << "Start state selected Grasp's part is " << grasp_poses[start->graspIndex().value].part_id << std::endl;
//    hystsp->printState(goal.get(), std::cout);
//    std::cout << "Goal state selected Grasp's part is " << grasp_poses[goal->graspIndex().value].part_id << std::endl;

//    double distance_btw_s_g = hystsp->distance(start.get(), goal.get());
    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

//    si->setValidStateSamplerAllocator(allocHybridValidStateSampler);
    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTConnect>(si));
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    planner->setRange(100.0);
    // perform setup steps for the planner
    planner->setup();
    // print the settings for this space
    si->printSettings(std::cout);
    // print the problem settings
    pdef->print(std::cout);
    // attempt to solve the problem within one second of planning time
    auto start_ts = std::chrono::high_resolution_clock::now();
    si->getStateSpace().get()->as<HybridObjectStateSpace>()->resetTimer();
    ob::PlannerStatus solved = planner->ob::Planner::solve(200.0);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time = finish - start_ts;

    if (solved)
    {
      if (pdef->hasExactSolution())
      {
        std::cout << "Has exact solution" << std::endl;
        succeeded_num += 1;
        double *total_time = new double;
        si->getStateSpace().get()->as<HybridObjectStateSpace>()->printExecutionDuration(total_time);
        running_time.push_back(*total_time);
        delete total_time;
      }
      else
      {
        std::cout << "Do not have exact solution" << std::endl;
        failed_num += 1;
      }

      og::PathGeometric slnPath = *(pdef->getSolutionPath()->as<og::PathGeometric>());
      // print the path to screen
      std::cout << "Found solution:\n" << "\n";
      std::cout << "Found solution with " << slnPath.getStateCount()
                << " states and length " << slnPath.length() << std::endl;
      // print the path to screen
      slnPath.printAsMatrix(std::cout);

      std::cout << "Writing PlannerData to file’./myPlannerData’" << std::endl;
      ob::PlannerData plannedData(si);
      planner->getPlannerData(plannedData);
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

    planner->clear();
  }
  PerformanceStats stats;
  stats.failed_num = failed_num;
  stats.succeeded_num = succeeded_num;
  getPerformanceStats(running_time, stats);
  return stats;
}

PerformanceStats twoHandoffTest(const ros::NodeHandle &node_handle,
                               const ros::NodeHandle &node_handle_priv,
                               const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
                               int num_test)
{
  std::string object_name = "needle_r";
  std::string robot_name = "robot_description";
  // create an instance of state space
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size(), grasp_poses));

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ompl::base::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.101);
  se3_xyz_bounds.setHigh(0, 0.101);
  se3_xyz_bounds.setLow(1, -0.06);
  se3_xyz_bounds.setHigh(1, 0.09);
  se3_xyz_bounds.setLow(2, 0.266);
  se3_xyz_bounds.setHigh(2, 0.496);

  hystsp->setSE3Bounds(se3_xyz_bounds);

  si->setStateValidityChecker(
    std::make_shared<HybridStateValidityChecker>(node_handle, robot_name, object_name, si));
  si->setMotionValidator(
    std::make_shared<HybridMotionValidator>(node_handle, node_handle_priv, robot_name, object_name, si));
  si->setup();

  ob::StateSamplerPtr stateSampler;  // setup a sampler
  stateSampler = si->allocStateSampler();  // assign HybridStateSampler to stateSampler

  int failed_num = 0;
  int succeeded_num = 0;
  std::vector<double> running_time;

  for (int i = 0; i < num_test; i++)
  {
    // create a random start state
    ob::ScopedState<HybridObjectStateSpace> start(hystsp);
    ob::ScopedState<HybridObjectStateSpace> goal(hystsp);

    bool is_ss_valid= false;
    while(!is_ss_valid)
    {
      stateSampler->sampleUniform(start.get());
      is_ss_valid = si->isValid(start.get());
    }

    stateSampler->sampleUniform(goal.get());

    int start_arm_index = start->armIndex().value;
    int start_grasp_part = grasp_poses[start->graspIndex().value].part_id;
    int goal_arm_index = goal->armIndex().value;
    int goal_grasp_part = grasp_poses[goal->graspIndex().value].part_id;
    bool same_arm = (start_arm_index == goal_arm_index) ? true : false;
    bool same_grasp_part = (start_grasp_part == goal_grasp_part) ? true : false;

    bool is_gs_valid = si->isValid(goal.get());
    while(!same_arm || same_grasp_part || !is_gs_valid)
    {
      stateSampler->sampleUniform(goal.get());
      is_gs_valid = si->isValid(goal.get());
      if(!is_gs_valid)
        continue;
      same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
      same_grasp_part = ( start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
    }

//    hystsp->printState(start.get(), std::cout);
//    std::cout << "Start state selected Grasp's part is " << grasp_poses[start->graspIndex().value].part_id << std::endl;
//    hystsp->printState(goal.get(), std::cout);
//    std::cout << "Goal state selected Grasp's part is " << grasp_poses[goal->graspIndex().value].part_id << std::endl;

//    double distance_btw_s_g = hystsp->distance(start.get(), goal.get());
    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

//    si->setValidStateSamplerAllocator(allocHybridValidStateSampler);
    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTConnect>(si));
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    planner->setRange(100.0);
    // perform setup steps for the planner
    planner->setup();
    // print the settings for this space
    si->printSettings(std::cout);
    // print the problem settings
    pdef->print(std::cout);
    // attempt to solve the problem within one second of planning time
    auto start_ts = std::chrono::high_resolution_clock::now();
    si->getStateSpace().get()->as<HybridObjectStateSpace>()->resetTimer();
    ob::PlannerStatus solved = planner->ob::Planner::solve(200.0);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time = finish - start_ts;

    if (solved)
    {
      if (pdef->hasExactSolution())
      {
        std::cout << "Has exact solution" << std::endl;
        succeeded_num += 1;
        double *total_time = new double;
        si->getStateSpace().get()->as<HybridObjectStateSpace>()->printExecutionDuration(total_time);
        running_time.push_back(*total_time);
        delete total_time;
      }
      else
      {
        std::cout << "Do not have exact solution" << std::endl;
        failed_num += 1;
      }

      og::PathGeometric slnPath = *(pdef->getSolutionPath()->as<og::PathGeometric>());
      // print the path to screen
      std::cout << "Found solution:\n" << "\n";
      std::cout << "Found solution with " << slnPath.getStateCount()
                << " states and length " << slnPath.length() << std::endl;
      // print the path to screen
      slnPath.printAsMatrix(std::cout);

      std::cout << "Writing PlannerData to file’./myPlannerData’" << std::endl;
      ob::PlannerData plannedData(si);
      planner->getPlannerData(plannedData);
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

    planner->clear();
  }
  PerformanceStats stats;
  stats.failed_num = failed_num;
  stats.succeeded_num = succeeded_num;
  getPerformanceStats(running_time, stats);
  return stats;
}

PerformanceStats threeHandoffTest(const ros::NodeHandle &node_handle,
                                 const ros::NodeHandle &node_handle_priv,
                                 const std::vector<cwru_davinci_grasp::GraspInfo>& grasp_poses,
                                 int num_test)
{
  std::string object_name = "needle_r";
  std::string robot_name = "robot_description";
  // create an instance of state space
  auto hystsp(std::make_shared<HybridObjectStateSpace>(1, 2, 0, grasp_poses.size(), grasp_poses));

  // construct an instance of  space information from this state space
  auto si(std::make_shared<ob::SpaceInformation>(hystsp));

  ompl::base::RealVectorBounds se3_xyz_bounds(3);
  se3_xyz_bounds.setLow(0, -0.101);
  se3_xyz_bounds.setHigh(0, 0.101);
  se3_xyz_bounds.setLow(1, -0.06);
  se3_xyz_bounds.setHigh(1, 0.09);
  se3_xyz_bounds.setLow(2, 0.266);
  se3_xyz_bounds.setHigh(2, 0.496);

  hystsp->setSE3Bounds(se3_xyz_bounds);

  si->setStateValidityChecker(
    std::make_shared<HybridStateValidityChecker>(node_handle, robot_name, object_name, si));
  si->setMotionValidator(
    std::make_shared<HybridMotionValidator>(node_handle, node_handle_priv, robot_name, object_name, si));
  si->setup();

  ob::StateSamplerPtr stateSampler;  // setup a sampler
  stateSampler = si->allocStateSampler();  // assign HybridStateSampler to stateSampler

  int failed_num = 0;
  int succeeded_num = 0;
  std::vector<double> running_time;

  for (int i = 0; i < num_test; i++)
  {
    // create a random start state
    ob::ScopedState<HybridObjectStateSpace> start(hystsp);
    ob::ScopedState<HybridObjectStateSpace> goal(hystsp);

    bool is_ss_valid= false;
    while(!is_ss_valid)
    {
      stateSampler->sampleUniform(start.get());
      is_ss_valid = si->isValid(start.get());
    }

    stateSampler->sampleUniform(goal.get());

    int start_arm_index = start->armIndex().value;
    int start_grasp_part = grasp_poses[start->graspIndex().value].part_id;
    int goal_arm_index = goal->armIndex().value;
    int goal_grasp_part = grasp_poses[goal->graspIndex().value].part_id;
    bool same_arm = (start_arm_index == goal_arm_index) ? true : false;
    bool same_grasp_part = (start_grasp_part == goal_grasp_part) ? true : false;

    bool is_gs_valid = si->isValid(goal.get());
    while(same_arm || !same_grasp_part || !is_gs_valid)
    {
      stateSampler->sampleUniform(goal.get());
      is_gs_valid = si->isValid(goal.get());
      if(!is_gs_valid)
        continue;
      same_arm = (start_arm_index == goal->armIndex().value) ? true : false;
      same_grasp_part = ( start_grasp_part == grasp_poses[goal->graspIndex().value].part_id) ? true : false;
    }

//    hystsp->printState(start.get(), std::cout);
//    std::cout << "Start state selected Grasp's part is " << grasp_poses[start->graspIndex().value].part_id << std::endl;
//    hystsp->printState(goal.get(), std::cout);
//    std::cout << "Goal state selected Grasp's part is " << grasp_poses[goal->graspIndex().value].part_id << std::endl;

//    double distance_btw_s_g = hystsp->distance(start.get(), goal.get());
    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

//    si->setValidStateSamplerAllocator(allocHybridValidStateSampler);
    // create a planner for the defined space
    auto planner(std::make_shared<og::RRTConnect>(si));
    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);
    planner->setRange(100.0);
    // perform setup steps for the planner
    planner->setup();
    // print the settings for this space
    si->printSettings(std::cout);
    // print the problem settings
    pdef->print(std::cout);
    // attempt to solve the problem within one second of planning time
    auto start_ts = std::chrono::high_resolution_clock::now();
    si->getStateSpace().get()->as<HybridObjectStateSpace>()->resetTimer();
    ob::PlannerStatus solved = planner->ob::Planner::solve(200.0);

    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> planning_time = finish - start_ts;

    if (solved)
    {
      if (pdef->hasExactSolution())
      {
        std::cout << "Has exact solution" << std::endl;
        succeeded_num += 1;
        double *total_time = new double;
        si->getStateSpace().get()->as<HybridObjectStateSpace>()->printExecutionDuration(total_time);
        running_time.push_back(*total_time);
        delete total_time;
      }
      else
      {
        std::cout << "Do not have exact solution" << std::endl;
        failed_num += 1;
      }

      og::PathGeometric slnPath = *(pdef->getSolutionPath()->as<og::PathGeometric>());
      // print the path to screen
      std::cout << "Found solution:\n" << "\n";
      std::cout << "Found solution with " << slnPath.getStateCount()
                << " states and length " << slnPath.length() << std::endl;
      // print the path to screen
      slnPath.printAsMatrix(std::cout);

      std::cout << "Writing PlannerData to file’./myPlannerData’" << std::endl;
      ob::PlannerData plannedData(si);
      planner->getPlannerData(plannedData);
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

    planner->clear();
  }
  PerformanceStats stats;
  stats.failed_num = failed_num;
  stats.succeeded_num = succeeded_num;
  getPerformanceStats(running_time, stats);
  return stats;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "handoff_main");
  ros::AsyncSpinner spinner(1);
  ros::Duration(3.0).sleep();
  spinner.start();

  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_priv("~");
  std::string object_name = "needle_r";
  std::string robot_name = "robot_description";

  cwru_davinci_grasp::DavinciSimpleNeedleGrasperPtr simpleGrasp =
    boost::make_shared<cwru_davinci_grasp::DavinciSimpleNeedleGrasper>(
      node_handle, node_handle_priv, "psm_one", object_name);

  std::vector<double> ss_needle_pose_translation;
  std::vector<double> ss_needle_pose_orientation;
  std::vector<double> gs_needle_pose_translation;
  std::vector<double> gs_needle_pose_orientation;
  std::vector<double> ss_joint_values;
  std::vector<double> gs_joint_values;
  int ss_arm_index;
  int ss_grasp_pose_index;
  int gs_arm_index;
  int gs_grasp_pose_index;

  if (node_handle_priv.hasParam("ss_needle_pose_translation"))
  {
    XmlRpc::XmlRpcValue needle_pose_list;
    node_handle_priv.getParam("ss_needle_pose_translation", needle_pose_list);

    ROS_ASSERT(needle_pose_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_pose_list.size(); ++i)
    {
      ROS_ASSERT(needle_pose_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      ss_needle_pose_translation.push_back(static_cast<double>(needle_pose_list[i]));
    }
  }

  if (node_handle_priv.hasParam("ss_needle_pose_orientation"))
  {
    XmlRpc::XmlRpcValue needle_ori_list;
    node_handle_priv.getParam("ss_needle_pose_orientation", needle_ori_list);

    ROS_ASSERT(needle_ori_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_ori_list.size(); ++i)
    {
      ROS_ASSERT(needle_ori_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      ss_needle_pose_orientation.push_back(static_cast<double>(needle_ori_list[i]));
    }
  }

  if (node_handle_priv.hasParam("gs_needle_pose_translation"))
  {
    XmlRpc::XmlRpcValue needle_pose_list;
    node_handle_priv.getParam("gs_needle_pose_translation", needle_pose_list);

    ROS_ASSERT(needle_pose_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_pose_list.size(); ++i)
    {
      ROS_ASSERT(needle_pose_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      gs_needle_pose_translation.push_back(static_cast<double>(needle_pose_list[i]));
    }
  }

  if (node_handle_priv.hasParam("gs_needle_pose_orientation"))
  {
    XmlRpc::XmlRpcValue needle_ori_list;
    node_handle_priv.getParam("gs_needle_pose_orientation", needle_ori_list);

    ROS_ASSERT(needle_ori_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < needle_ori_list.size(); ++i)
    {
      ROS_ASSERT(needle_ori_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      gs_needle_pose_orientation.push_back(static_cast<double>(needle_ori_list[i]));
    }
  }

  if (node_handle_priv.hasParam("ss_joint_values"))
  {
    XmlRpc::XmlRpcValue joint_value_list;
    node_handle_priv.getParam("ss_joint_values", joint_value_list);

    ROS_ASSERT(joint_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < joint_value_list.size(); ++i)
    {
      ROS_ASSERT(joint_value_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      ss_joint_values.push_back(static_cast<double>(joint_value_list[i]));
    }
  }

  if (node_handle_priv.hasParam("gs_joint_values"))
  {
    XmlRpc::XmlRpcValue joint_value_list;
    node_handle_priv.getParam("gs_joint_values", joint_value_list);

    ROS_ASSERT(joint_value_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < joint_value_list.size(); ++i)
    {
      ROS_ASSERT(joint_value_list[i].getType() ==
                 XmlRpc::XmlRpcValue::TypeDouble);
      gs_joint_values.push_back(static_cast<double>(joint_value_list[i]));
    }
  }

  node_handle_priv.getParam("ss_arm_index", ss_arm_index);
  node_handle_priv.getParam("ss_grasp_pose_index", ss_grasp_pose_index);
  node_handle_priv.getParam("gs_arm_index", gs_arm_index);
  node_handle_priv.getParam("gs_grasp_pose_index", gs_grasp_pose_index);

  std::vector <cwru_davinci_grasp::GraspInfo> grasp_poses = simpleGrasp->getAllPossibleNeedleGrasps(true);

  PerformanceStats oneHfStats = oneHandoffTest(node_handle, node_handle_priv, grasp_poses, 100);
  PerformanceStats twoHfStats = twoHandoffTest(node_handle, node_handle_priv, grasp_poses, 100);
  PerformanceStats threeHfStats = threeHandoffTest(node_handle, node_handle_priv, grasp_poses, 100);

  printPerformanceStats(oneHfStats, "One Handoff");
  printPerformanceStats(twoHfStats, "Two Handoff");
  printPerformanceStats(threeHfStats, "Three Handoff");

  return 0;
}
