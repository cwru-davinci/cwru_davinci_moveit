This package stores MoveIt configuration setup of daVinci dual-PSMs(Patient Side Manipulators).

Developed by [Su Lu](https://github.com/lusu8892/) at the Mercis Lab, Case Western Reserve University.

## Install

### Ubuntu Debian
Kinetic:
```
git clone https://github.com/lusu8892/cwru_davinci_moveit
```

### You would like to download daVinci dual-PSMs' URDF file from here
```
git clone https://github.com/cwru-davinci/uv_geometry 
```

## How to use

To use as mode of kinematics simulation only. This mode does not require pairing with either physical robot or Gazebo simulated robot
```
roslaunch cwru_davinci_dvrk_both_psms_moveit_config demo.launch
```

To uas as mode of full simulation or of running with physical robot
```
# first launch simulated robot or physical robot
roslaunch cwru_davinci_dvrk_both_psms_moveit_config bringup.launch
```
