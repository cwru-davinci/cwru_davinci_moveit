This package stores MoveIt configuration setup of daVinci dual-PSMs(Patient Side Manipulators).

Developed by [Su Lu](https://github.com/lusu8892/) at the MeRCIS Lab, Case Western Reserve University.

## Install

### Ubuntu
Kinetic:
```
git clone https://github.com/cwru-davinci/cwru_davinci_moveit
git clone https://github.com/cwru-davinci/uv_control
git clone https://github.com/cwru-davinci/sim_gazebo
```

### You would like to download daVinci dual-PSMs' URDF file from here
```
git clone https://github.com/cwru-davinci/uv_geometry 
```

## How to use

To use as mode of **Kinematics Simulation** only. This mode does not require pairing with either physical robot or Gazebo simulated robot
```
roslaunch cwru_davinci_dvrk_both_psms_moveit_config demo.launch
```

To uas as mode of **Full Simulation** or of running with **Physical Robot**
```
# Launch simulated robot or physical robot
# For simulation, first launch
roslaunch sim_gazebo launch_from_config.launch

# Then launch
roslaunch cwru_davinci_dvrk_both_psms_moveit_config bringup.launch
```
