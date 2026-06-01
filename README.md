# IsaacSim Surgical Robotics Challenge for ICRA 2026

This branch contains the Isaac Sim implementation of the [AMBF Surgical Robotics Challenge](https://github.com/surgical-robotics-ai/surgical_robotics_challenge) developed by the Johns Hopkins LCSR Lab.

This implementation has been updated to use ROS2 on Ubuntu 24.04, and provides the pegboard and pegs used for the [ICRA 2026 competition](https://surgical-robotics-ai.github.io/icra-competition-2026/index.html).

## Installation

1. Install ROS2 Jazzy and Isaac Sim

Ensure that the correct version of ROS 2 is installed and sourced on your system. ROS2 Jazzy can be found [here](https://docs.ros.org/en/jazzy/Installation.html).

Isaac Sim can be downloaded from [NVIDIA's website](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html).

2. Clone this repository

Refer to README in [Assets](assets/README.md) and [Scripts](Scripts/README.md) for further information.

## Running the simulation

1. Open a terminal and set up for ROS 2: `source /opt/ros/jazzy/setup.bash`

2. Change to the `isaac-sim` directory and start Isaac Sim version 5.1.0:

```
cd ~/isaac-sim
./isaac-sim.sh
```

3. Open the file `ICRA_Challenge.usd`. You should see the following scene in the view monitor:

![Image](https://github.com/surgical-robotics-ai/isaac-sim-surgical-robotics-challenge/blob/ICRA-Challenge-2026/Media/pegboard.png)

4. Press the `play' button (or press the spacebar) to play the Isaac Sim Simulation

## Teleoperating the simulation

After completing the steps above to run the simulation:

1. Open a new terminal and set up for ROS 2: `source /opt/ros/jazzy/setup.bash`

2. Change to the teleoperation directory:

~~~
cd isaac-sim-surgical-robotics-challenge/scripts/surgical_robotics_challenge/teleoperation
~~~

3. Run the teleoperation control using the following command line:

~~~
python udp_crtk_bridge.py
~~~

This control script converts CRTK-compatible JSON commands sent/received via UDP from the master device (e.g., Quest 3) to CRTK-compatible ROS2 topics for IsaacSim.
