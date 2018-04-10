# cwru_davinci_both_psms_moveit_config

This package is to perform path planning on Two da Vinci Patient Side Manipulators (PSMs) of dvrk (da Vinci Research Kit), which is carried out by [MoveIt!](https://moveit.ros.org/) motion planning library.

## Install

To have this package working with, there are packages need to be installed.

### Ubuntu Debian

For ROS Kinetic, install MoveIt! pacakge at first.

```
sudo apt-get install ros-kinetic-moveit
```

Then install [MoveIt! Visual Tools](https://github.com/ros-planning/moveit_visual_tools).

```
sudo apt-get install ros-kinetic-moveit-visual-tools
```

To execute MoveIt! planned path correctly in Gazbebo simluation, user needs to install [MimicJointPlugin](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins) created by [Robotics Group University of Patras](https://github.com/roboticsgroup).

```
cd /home/username/ros_workspace/src
git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
catkin build
```

## Visualizing path planning by MoveIt! RViz Plugin

```
roslaunch cwru_davinci_both_psms_moveit_config demo.launch
```

The way about how to use RViz GUI to perform path planning on PSMs, please refer to [MoveIt! RViz Plugin Tutorial](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/ros_visualization/visualization_tutorial.html)

## Running MoveIt! planned path in Gazebo simulation

```
roslaunch cwru_davinci_both_psms_moveit_config dvrk_psm_both_sim.launch
```

Once Gazebo finished loading both PSMs model, bring up Rviz next.

```
roslaunch cwru_davinci_both_psms_moveit_config bringup.launch
```
