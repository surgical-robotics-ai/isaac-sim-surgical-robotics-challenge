# IsaacSim Surgical Robotics Challenge for ICRA 2026

This branch contains the Isaac Sim implementation of the [AMBF Surgical Robotics Challenge](https://github.com/surgical-robotics-ai/surgical_robotics_challenge) developed by the Johns Hopkins LCSR Lab.

This implementation has been updated to use ROS2 on Ubuntu 24.04, and provides the pegboard and pegs used for the [ICRA 2026 competition](https://surgical-robotics-ai.github.io/icra-competition-2026/index.html).

## 1. Install ROS2 Jazzy and Isaac Sim

Ensure that the correct version of ROS 2 is installed and sourced on your system. ROS2 Jazzy can be found [here](https://docs.ros.org/en/jazzy/Installation.html).

Isaac Sim can be downloaded from [NVIDIA's website](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html).

## 2. Clone this repository

Refer to README in [Assets](assets/README.md) and [Scripts](Scripts/README.md) for further information.

## 3. Run the Simulation

1. After installation of all prerequisite programs, open Isaac Sim version 5.1.0

2. Open the file `ICRA_Challenge.usd`. You should see the following scene in the view monitor:

![Image](https://github.com/surgical-robotics-ai/isaac-sim-surgical-robotics-challenge/blob/ICRA-Challenge-2026/Media/pegboard.png)

3. Navigate to preferred ROS 2 teleoperation controller (Quest for ICRA 2026 challenge)

~~~
cd Isaac-Sim-Surgical-Robotics-Challenge/scripts/surgical_robotics_challenge/teleoperation
~~~

4. Play Isaac Sim Simulation
5. Source the running terminal with ROS 2: `source /opt/ros/jazzy/setup.bash`
6. Run desired teleoperation control with the following command line

~~~
python3 [controller].py -c isaac_sim --three false
~~~

## Other input devices (not available at ICRA 2026):

Repositories for other control devices can be found in the links below for MTM and Phantom Omni:

**Master Tool Manipulator (MTM):**
[https://github.com/jhu-dvrk/dvrk-ros](https://github.com/jhu-dvrk/dvrk-ros)

**Phantom Omni:**
[https://github.com/WPI-AIM/ros_geomagic](https://github.com/WPI-AIM/ros_geomagic)
