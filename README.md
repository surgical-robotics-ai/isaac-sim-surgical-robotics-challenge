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
1. After installation of all prerequisite programs, open Isaac Sim version 4.5.0
2. Navigate to file and open either version of the Surgical_Challenge.usd (Surgical_Robotics_Challenge_New or Surgical_Robotics_Challenge_Old)
You should see the following scene in the view monitor

![Image](https://github.com/tkim104/Isaac-Sim-Surgical-Robotics-Challenge/blob/main/Media/viewport.png)

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

## 4. Choosing Thread Type
From Isaac Sim 4.0.0 and on, you are able to activate and deactivate different assets within the scene. To activate another thread in the scene, see below:
1. Rigid Body Thread: Set Physic Scene step to 300, Activate "Needle_Thread" and deactivate others. Turn OFF GPU in physics scene.
2. Deformable Body Thread: Set Physic Scene step to 60 (default), Activate "Needle_Deformable" and deactivate others. Turn ON GPU in physics scene.
3. Particle Body Thread: Set Physic Scene step to 60 (default), Activate "Needle_Particle" and deactivate others. Turn ON GPU in physics scene.

## 5. Input Devices
Repositories for other control devices can be found in the links below for MTM and Phantom Omni

**Master Tool Manipulator (MTM)**
a. https://github.com/jhu-dvrk/dvrk-ros

**Phantom Omni**
b. https://github.com/WPI-AIM/ros_geomagic

## 6. Example
![Video](https://github.com/tkim104/Isaac-Sim-Surgical-Robotics-Challenge/blob/main/Media/ISMR25_0023_VD_i.mp4)
