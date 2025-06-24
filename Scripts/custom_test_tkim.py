from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import sys

import carb
import omni
from pxr import Gf
import numpy as np
from isaacsim.core.api import World
import omni.isaac.core.utils.prims as prims_utils
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units, print_stage_prim_paths
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.simulation_manager import SimulationManager
from argparse import ArgumentParser

# enable ROS bridge extension
enable_extension("isaacsim.ros1.bridge")
enable_extension("omni.new.extension")

simulation_app.update()

# check if rosmaster node is running
# this is to prevent this sample from waiting indefinetly if roscore is not running
# can be removed in regular usage
import rosgraph

if not rosgraph.is_master_online():
    carb.log_error("Please run roscore before executing this script")
    simulation_app.close()
    exit()

# Note that this is not the system level rospy, but one compiled for omniverse
import rospy
from nav_msgs.msg import Odometry

class SubscribeCamera:
    def __init__(self, world):
        # setup the ros subscriber here
        self.ros_sub = rospy.Subscriber("/CameraFramePublisher", Odometry, self.update_camera_callback, queue_size=10)
        self.camera_position = Gf.Vec3d(0.0, 0.0, 0.0) # Starting Position
        self.camera_orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0) # Starting Orientation
        self.ros_world = world

    def update_camera_callback(self, data):
        # callback function to set the cube position to a new one upon receiving a ros message
        if self.ros_world.is_playing():
            self.camera_position = Gf.Vec3d(data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
            self.camera_orientation = Gf.Quatd(data.pose.pose.orientation.w, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z)

if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-p', action='store', dest='challenge_path', help='Path to Isaac Sim Surgical Robotics Challenge', default=None)
    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    asset_path = parsed_args.challenge_path
    if asset_path is None:
        carb.log_error("Please specify challenge path")
        simulation_app.close()
        exit()

    # preparing the scene
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        simulation_app.close()
        sys.exit()

    world = World(stage_units_in_meters=1.0)
    set_camera_view(
        eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp"
    )  # set camera view

    # Enable GPU dynamics with Isaac Sim Simulation Manager
    simulation_manager = SimulationManager()
    simulation_manager.enable_gpu_dynamics(True)

    # Add Isaac Sim Surgical Challenge Reference
    #asset_path = "/home/armone/Desktop/Isaac-Sim-Surgical-Robotics-Challenge-main/Assets/Surgical_Challenge_Old_Phantom.usd"
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/Test")  # add robot to stage

    # Begin Simulation Timeline
    timeline = omni.timeline.get_timeline_interface()

    # initialize the world
    world.reset()

    rospy.init_node("Standalone", anonymous=True, disable_signals=True, log_level=rospy.ERROR)
    timeline.play()
    ECM = SubscribeCamera(world)
    reset_needed = False
    
    # Get Isaac Sim Camera Prim
    ecm_camera = prims_utils.get_prim_at_path("/World/Test/ECM/Cube")

    while simulation_app.is_running():
            world.step(render=True)
            if world.is_stopped() and not reset_needed:
                reset_needed = True
            if world.is_playing():
                if reset_needed:
                    world.reset()
                    reset_needed = False
                # the actual setting the cube pose is done here
                ecm_camera.GetAttribute("xformOp:translate").Set(ECM.camera_position)
                ecm_camera.GetAttribute("xformOp:orient").Set(ECM.camera_orientation)

    # Cleanup
    ECM.ros_sub.unregister()
    rospy.signal_shutdown("subscriber example complete")
    timeline.stop()
    simulation_app.close()
