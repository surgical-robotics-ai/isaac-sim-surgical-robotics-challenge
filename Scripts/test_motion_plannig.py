from surgical_robotics_challenge.utils import interpolation
import numpy as np

interpolate = interpolation.Interpolation()
x0 = np.array([0])
xf = np.array([1])
dx0 = np.array([0])
dxf = np.array([2])
ddx0 = np.array([0])
ddxf = np.array([3])
interpolate.compute_interpolation_params(x0, xf, dx0, dxf, ddx0, ddxf, 0, 5)
interpolate.plot_trajectory(t0 = 0, tf = 5)

# You create an interpolation class to calculate interim position, velocity, and acceleration
# in your direct motion path. You provide starting and final position, velocity, and acceleration
# for the class to calculate the route. Given a starting and final time steps, you subdivide
# the time step by N to calcualte the inbetween movements

# plot_trajectory provides an example on how to use the methods in the class to get motion
# planning values in cartesian space

# Isaac Sim controller uses Joint Space to communicate in ROS1, use inverse and forward kinematics
# to translate the position, velocity, and acceleration to joint angles for Isaac Sim to follow

# TASK 1: Move PSM to a needle
# TASK 2: Grasp needle
# TASK 3: Raise needle (Reposition and grasp again if necessary)
# TASK 4: Move needle to suturing point
# TASK 5: Circular movement motion planning to get neeldle through the other side
# TASK 6: Grasp needle from the other side
# TASK 7: Pull needle out to the other side of suture

# The goal is to use the same motion planning algorithm to move the needle through the surgical
# environment in both AMBF and Isaac Sim to show comparison
# This will require more work on Isaac Sim to provide an accurate / better surgical environment

# 3 Main Tasks left to do
# 1: Motion Planner / Example movement of Isaac Sim and AMBF
# 2: Deformable Phantom + Improved Collision Detection in Isaac Sim
# 3: AMBF Latency Value Comparison