# Isaac-Sim-Surgical-Robotics-Challenge
Isaac Sim implementation of the AMBF Surgical Robotics Challenge developed by Johns Hopkins LCSR Lab.
This implementation has been developed and tested on Ubuntu 20.04

See: https://github.com/surgical-robotics-ai/surgical_robotics_challenge
To find the work that this project is based on. 

## 1. Install AMBF, ROS1 Noetic, and Isaac Sim
Ensure that the correct version of ROS is installed and sourced on your system. ROS1 Noetic can be found here: 
http://wiki.ros.org/noetic/Installation/Ubuntu

AMBF and directions for installation can be found here: 
https://github.com/WPI-AIM/ambf

Isaac Sim can be downloaded here from NVIDIA's wesbsite: 
https://developer.nvidia.com/isaac/sim

For additional resources, see Documents for summarized installation steps and notes for installations.

## 2. Clone Repository
Refer to README in Assets and Scripts to access Isaac Sim Surgical Challenge for use

## 3. Running Simulation
1. After installation of all prerequisite programs, open Isaac Sim version 4.1.0
2. Navigate to file and open 1 of 4 versions Surgical_Challenge.usd
You should see the following scene in the view monitor

![Image]()

3. Start roscore in a seperate terminal
4. Navigate to preferred ROS operated controller

~~~
cd Isaac-Sim-Surgical-Robotics-Challenge/scripts/surgical_robotics_challenge/teleoperation
~~~

5. Play Isaac Sim Simulation
6. Run desired teleoperation control with the following command line

~~~
python3 [controller].py -c isaac_sim --three false
~~~

## 4. Input Devices
Repositories for other control devices can be found in the links below for MTM and Phantom Omni

**Master Tool Manipulator (MTM)**
a. https://github.com/jhu-dvrk/dvrk-ros

**Phantom Omni**
b. https://github.com/WPI-AIM/ros_geomagic

