from ambf_client import Client
from .isaac_client import IsaacClient
import surgical_robotics_challenge.units_conversion as units_conversion
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import math
from PyKDL import Vector, Rotation, Frame

class SimulationObject:
    def __init__(self, ambf_object):
        self._object = ambf_object
        self._joint_type = None # To distinguish between revolute and prismatic joints

    def set_joint_types(self, joint_types):
        self._joint_type = joint_types

    def get_pos(self):
        return units_conversion.get_pos(self._object)

    def get_rotation(self):
        return units_conversion.get_rotation(self._object)

    def get_pose(self):
        return units_conversion.get_pose(self._object)

    def set_pos(self, pos):
        units_conversion.set_pos(self._object, pos)

    def set_pose(self, pose):
        units_conversion.set_pos(self._object, pose.p)
        self.set_rotation(pose.M)

    def set_rpy(self, r, p, y):
        units_conversion.set_rpy(self._object, r, p, y)

    def set_rotation(self, R):
        rpy = R.GetRPY()
        units_conversion.set_rpy(self._object, rpy[0], rpy[1], rpy[2])

    def get_joint_pos(self, idx):
        return units_conversion.get_joint_pos(self._object, idx, self._joint_type[idx])

    def set_joint_pos(self, idx, cmd):
        units_conversion.set_joint_pos(self._object, idx, self._joint_type[idx], cmd)

    def get_joint_vel(self, idx):
        return units_conversion.get_joint_vel(self._object, idx, self._joint_type[idx])

    def set_joint_vel(self, idx, cmd):
        units_conversion.set_joint_vel(self._object, idx, self._joint_type[idx], cmd)

    def get_joint_names(self):
        return self._object.get_joint_names()

    def set_force(self, f):
        self._object.set_force(f[0], f[0], f[0])

    def set_torque(self, t):
        self._object.set_torque(t[0], t[0], t[0])


class SimulationManager:
    def __init__(self, name):
        self._client = None
        if name == "isaac_sim":
            self._client = IsaacClient(name)
        else:
            self._client = Client(name)
        self._client.connect()

    def get_obj_handle(self, name):
        object = self._client.get_obj_handle(name)
        if object:
            return SimulationObject(object)
        else:
            return None

    def get_world_handle(self):
        return self._client.get_world_handle()

"""
class IsaacObject:
    def __init__(self, isaac_object):
        if isaac_object == "CameraFrame":
            self.sub = rospy.Subscriber(isaac_object, Odometry, self.callback)
        if isaac_object == "psm1/baselink":
            self.sub = rospy.Subscriber(isaac_object, Odometry, self.callback)
        self.pose_message = Pose
        self._joint_type = None # To distinguish between revolute and prismatic joints
        self.pose = None

    def callback(self, data):
        self.pose_message = data.pose.pose
        self.pose = self.pose_conversion()

    def quaternion_to_euler(self, orientation):
        # Extract the values from Orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def pose_conversion(self):
        v = Vector(self.pose_message.position.x, self.pose_message.position.y, self.pose_message.position.z)
        roll, pitch, yaw = self.quaternion_to_euler(self.pose_message.orientation)
        r = Rotation.RPY(roll, pitch, yaw)
        return Frame(r, v)

    def get_pose(self):
        return self.pose
    
    def set_pose(self, pose):
        self.pose = pose

    def set_joint_types(self, joint_types):
        self._joint_type = joint_types

    def set_joint_pos(self, idx, cmd):
        pass
        #units_conversion.set_joint_pos(self._object, idx, self._joint_type[idx], cmd)

class IsaacSimulationManager:
    def __init__(self, name):
        self._client = Client(name)
        self._client.connect()

    def get_obj_handle(self, name):
        isaac_object = name
        if isaac_object:
            return IsaacObject(name)
        else:
            None
"""
