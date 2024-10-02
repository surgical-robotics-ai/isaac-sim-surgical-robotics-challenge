# Scripts Info
The scripts listed and used in this project are based off the AMBF Surgical Challenge found [here](https://github.com/surgical-robotics-ai/surgical_robotics_challenge). Many of the existing utilities and teleoperation scripts are direct copies from the project and explanations can be found in the AMBF repository.

## Controlling Simulated Robots:
Before launching Isaac Sim, ensure to activate the ROS1 bridge for your version. 
![Image](https://github.com/tkim104/Isaac-Sim-Surgical-Robotics-Challenge/blob/main/Media/startisaac.png)

Start roscore in another terminal or command-line before starting the simulation in the Surgical_Challenge.usd files. Roscore must be activated before starting the simulation and the Isaac Sim simulation must be running before running any teleoperation script. Using rostopics, Isaac Sim will read and send the PSM joint states in the dVRK-CRTK rostopic format.

## Isaac Sim Omnigraph Custom Nodes
Isaac Sim allows users to create custom interactive scripts with their Action Graph / Omnigraph. This feature is currently only present in version 2023.1, but nodes created this way does work for later versions.

To enable the custom Omnigraph nodes, download the Kit folder listed in scripts and add it to your Documents folder. In Isaac Sim, enable the custom extensions and autoload to use the custom nodes for the Surgica Challenge files.

![Image](https://github.com/tkim104/Isaac-Sim-Surgical-Robotics-Challenge/blob/main/Media/custom_nodes.png)

## Isaac Sim Scripts
Several key files added or modified from the AMBF Surgical Challenge:

#### 1. Additional Files
| # | File Name                | Description   |
|---|:------------------------:|:-------------:|
| 1 | isaac_sim_base_object.py | Modeled after the AMBF base_object, this file creates an object with a rostopic to communicate between Isaac Sim, ROS1, and Teleoperation Scripts|
| 2 | isaac_sim_camera.py      | Camera object that holds camera rostopic object| 
| 3 | isaac_sim_psm.py         | PSM object that holds PSM rostopic object|
| 4 | watch_dog.py             | Root base object to communicate between Isaac Sim and ROS1 |
| 5 | isaac_client.py          | Primary file that acts between Isaac Sim and ROS1 to mediate the pipeline between the two applications. Must be called when defining which program any teleoperation script wishes to communicate with, modeled off of the ambf_client defined in the AMBF Surgical Challenge |

#### 2. Modified Files
| # | File Name             | Description   |
|---|:---------------------:|:-------------:|
| 1 | simulation_manager.py | Modified to detect whether the isaac_sim client has been called, appropriately chooses between AMBF and Isaac Sim simulation manager |
| 2 | units_conversion.py   | Modified to match 1-for-1 scaling between a real PSM and simulated PSM |

## Teleoperation
All teleoperation files are identical to the ones found in the AMBF Surgical Challenge. Thus, teleoperation scripts based off of AMBF will work for Isaac Sim using the other client.

#### Scripts
| # | File Name                     | Description   |
|---|:-----------------------------:|:-------------:|
| 1 | gui_based_control.py          | Gui based controller allows GUI to control multiple PSMs at the same time with n number of windows (depending on how many PSMs are requested to control) |
| 2 | geomagic_multi_psm_control.py | Uses a Phantom Omni Geomagic device to acquire the pose and orientations of the control for virtual PSM control. Double-tapping the dark grey button allows one to switch control between multiple PSMs |
| 3 | mtm_multi_psm_control.py      | Uses a mtm device and psm arms to bind a single Master Tool Manipulator (MTM) to multiple Patient Side Manipulators (PSMs) Translating across the virtual space can be used with the clutch pedal |

## Additional Notes / Troubleshooting
1. Depending on your version of Isaac Sim, a GPU / CUDA failure requires a complete restart of the machine to control and move assets within the Isaac Sim Surgical Challenge.
2. If joint indexes between Isaac Sim and the Teleoperation script do not align (e.g. Joint IDX [0 - -1]) restart Isaac Sim and follow startup protocol again.
3. Teleoperation scripts will not start if roscore and Isaac Sim are not running (and simulating), however, teleoperation control continues even if Isaac Sim simulation stops and thus does not need to restart teleoperation script (will need restart if roscore is turned off at any point).
