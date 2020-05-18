The package is for daVinci to do dual arm manipulation. More specifically it is now for needle handoff project.

This dual arm manipulation planner is implemented by [**RRT-CONNECT**](https://www.cs.cmu.edu/afs/cs/academic/class/15494-s14/readings/kuffner_icra2000.pdf). The RRT-CONNECT code base is from [**OMPL**](https://ompl.kavrakilab.org/). Robot description, collision check, and kinematics are done by using [**MoveIt**](https://moveit.ros.org/).

This package includes:

  - [**HybridObjectStateSpace**](https://github.com/lusu8892/cwru_davinci_moveit/blob/e795114ebeb192d5bb612ea680586cf373dcf74f/cwru_davinci_moveit_planners/cwru_davinci_dual_arm_manipulation_planner/dual_arm_manipulation_planner_interface/include/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h#L98)
defines a state space which is composed by cartesian space and two discrete state spaces. The space is used to have RRT-CONNECT work on it.
  - [**HybridStateSampler**](https://github.com/lusu8892/cwru_davinci_moveit/blob/e795114ebeb192d5bb612ea680586cf373dcf74f/cwru_davinci_moveit_planners/cwru_davinci_dual_arm_manipulation_planner/dual_arm_manipulation_planner_interface/include/dual_arm_manipulation_planner_interface/parameterization/hybrid_object_state_space.h#L67)
defines a hybrid state space sampler that samples a random state.
  - [**HybridStateValidityChecker**](https://github.com/lusu8892/cwru_davinci_moveit/blob/e795114ebeb192d5bb612ea680586cf373dcf74f/cwru_davinci_moveit_planners/cwru_davinci_dual_arm_manipulation_planner/dual_arm_manipulation_planner_interface/include/dual_arm_manipulation_planner_interface/hybrid_state_validity_checker.h#L58)
defines a validity checker which is used to verify if state is valid also check if robot state is in collision.
  - [**HybridMotionValidator**](https://github.com/lusu8892/cwru_davinci_moveit/blob/e795114ebeb192d5bb612ea680586cf373dcf74f/cwru_davinci_moveit_planners/cwru_davinci_dual_arm_manipulation_planner/dual_arm_manipulation_planner_interface/include/dual_arm_manipulation_planner_interface/hybrid_motion_validator.h#L54)
defines a local planner which is used to connect two hybrid states.
  - [**HybridObjectHandoffPlanner**](https://github.com/lusu8892/cwru_davinci_moveit/blob/e795114ebeb192d5bb612ea680586cf373dcf74f/cwru_davinci_moveit_planners/cwru_davinci_dual_arm_manipulation_planner/dual_arm_manipulation_planner_interface/include/dual_arm_manipulation_planner_interface/hybrid_object_handoff_planner.h#L71)
defines a object handoff planner which is used to do needle handoff path planning.
  - [**DavinciNeedleHandoffExecutionManager**](https://github.com/lusu8892/cwru_davinci_moveit/blob/e795114ebeb192d5bb612ea680586cf373dcf74f/cwru_davinci_moveit_planners/cwru_davinci_dual_arm_manipulation_planner/dual_arm_manipulation_planner_interface/include/dual_arm_manipulation_planner_interface/davinci_needle_handoff_execution_manager.h#L52)
controls daVinci dual-PSMs to do needle handoff motion which trajectories is calculated from **HybridObjectHandoffPlanner**.

Developed by [Su Lu](https://github.com/lusu8892/) at the Mercis Lab, Case Western Reserve University.

## Install
Before downloading code from here, please make sure you have installed MoveIt and OMPL from **source** **only**!

### Ubuntu Debian
Kinetic:
```
git clone https://github.com/lusu8892/cwru_davinci_moveit
```

### Some other packages needed before compiling
```
git clone https://github.com/lusu8892/cwru_davinci_grasp
git clone https://github.com/cwru-davinci/uv_control.git
git clone https://github.com/cwru-robotics/cwru_davinci_kinematics.git
git clone https://github.com/cwru-davinci/sim_gazebo.git
```

## How to use
Before use, make sure the updated needle pose is being published to topic "/updated_needle_pose", or remap whatever topic name to this one, the data type should be "geometry_msgs/PoseStamped".

To use in simulation
```
roslaunch sim_gazebo launch_from_config.launch
roslaunch cwru_davinci_dvrk_both_psms_moveit_config bringup.launch
roslaunch cwru_davinci_dual_arm_manipulation_planner davinci_needle_handoff_execution_main_simulation.launch
```

To use on hardware
```
roslaunch cwru_davinci_dvrk_both_psms_moveit_config bringup.launch
roslaunch cwru_davinci_dual_arm_manipulation_planner davinci_needle_handoff_execution_main_hardware.launch
```

## TODO

Furture features to be added to this project:
