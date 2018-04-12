# cwru_davinci_moveit_object_handling

This package is meant to generates MoveIt! *moveit_msgs/CollisionObject* messages from arriving *object_msgs/Object* messages.

## Install
To have this package working, there are some external packages need to be installed.

```
cd /your_ros_workspace/src
git clone https://github.com/JenniferBuehler/convenience-pkgs.git
git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
git clone https://github.com/JenniferBuehler/general-message-pkgs.git
catkin build
```

## Package usage guide:

### Step1: Load required parameters for object information service and fake object recognition
Firstly, user needs to set the required ROS parameters for loading the Gazebo "World plugin" (see also this [gazebo-pkgs wiki page](https://github.com/JenniferBuehler/gazebo-pkgs/wiki/Gazebo-world-plugins)). This is in need when Gazebo is launched. Meanwhile, this launch file loads up the MoveIt! collision object generator:

```
roslaunch cwru_davinci_moveit_object_handling gazebo_object_recognition.launch
```

### Step2: Start up Gazebo, MoveIt! and RVIz
Firstly, start Gazebo simluation with dvrk both_psms

```
roslaunch cwru_davinci_both_psms_moveit_config dvrk_psm_both_sim.launch
```

Then, start MoveIt! along with Rviz

```
roslaunch cwru_davinci_both_psms_moveit_config bringup.launch
```

### Step3:  Spawn a needle in Gazebo
For needle spawning, user just needs to inset a needle model into gazebo and setup its pose properly with respect to both_psms.

### Step4: Recognize the needle
So by now, user should be able to have needle recognized and have MoveIt! with needle's collision model. This recognition is a "fake" one which means its node consistently "recognises" the object in scene but without any senor and then publishes the object info to MoveIt!.

```
rosrun gazebo_test_tools fake_object_recognizer_cmd <needle's name in Gazebo> 1
```

### Step5: Check needle's collision model in RViz
Once the needle has been "recognized", its collision model will be seen in RViz in bright green color at the exact same pose as it is in Gazebo.

## TODO
For future simulation, some features will be added to this project:

 - Replace the "fake recognizer" with real object recognition program.
 - Spawing needle or other object by running a ros node.

## Acknowledgement
Thanks for [JenniferBuehler](https://github.com/JenniferBuehler) on Github.
For detail please refer to [the wiki](https://github.com/JenniferBuehler/moveit-pkgs/wiki) for more information.
