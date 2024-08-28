from .isaac_sim_base_object import BaseObject
from transformations import quaternion_from_euler, euler_from_quaternion
import rospy
import math

# Isaac Sim based conversion of AMBF Rigid Body for use in simulator / rostopic communication
# Specifically made for Isaac Sim's version of a PSM object
class PSM(BaseObject):
    def __init__(self, a_name, time_out=0.1):
        """
        Constructor
        :param a_name:
        """
        super(PSM, self).__init__(a_name, time_out)  # Set duration of Watchdog expiry
        self.object_type = "RIGID_BODY"
        self.body_type = "DYNAMIC"
        self._wrench_cmd_set = False  # Flag to check if a Wrench command has been set
        self._pose_cmd_set = False  # Flag to check if a Pose command has been set
        self._twist_cmd_set = False  # Flag to check if a Twist command has been set
        self._subj = None
        self._joints = None
        self._joints_temp = None

    def _apply_command(self):
        """
        Internal function to synchronized with the publisher and update watchdog
        :return:
        """
        self._cmd.header.stamp = rospy.Time.now()
        #print(self._cmd)
        #self._pub.publish(self._cmd)
        self.acknowledge_wd()

    def ros_j_cb(self, data):
        """
        Call function for ROS topics
        :param data:
        :return:
        """
        self._joints = data
        if self._joints_temp is None:
            self._joints_temp = self._joints


    def is_joint_idx_valid(self, joint_idx):
        """
        :param joint_idx:
        :return:
        """
        n_jnts = len(self._joints.position)
        if 0 <= joint_idx < n_jnts:
            return True
        else:
            # Index invalid
            print('ERROR! Requested Joint Idx of \"' + str(joint_idx) +
                  '\" outside valid range [0 - ' + str(n_jnts - 1) + ']')
            return False

    def get_joint_idx_from_name(self, joint_name):
        """
        :param joint_name:
        :return:
        """
        joint_names = self._state.joint_names
        if joint_name in joint_names:
            joint_idx = joint_names.index(joint_name)
            return joint_idx
        else:
            print('ERROR! Requested Joint \"' + str(joint_name) + '\" not found in list of joints:')
            print(joint_names)
            return None

    def get_rpy(self):
        """
        Get the rotation as Fixed RPY for this object
        :return:
        """
        quat = self._state.pose.pose.orientation
        rpy = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return rpy
    
    def get_pos(self):
        """
        Get the position in the parent frame for this object in parent frame
        :return:
        """
        return self._state.pose.pose.position

    def _set_joint_command(self, joint_name_or_idx, cmd, cmd_type, apply_command=True):
        """
        :param joint_name_or_idx:
        :param cmd: COMMAND VALUE
        :param cmd_type: FORCE, VELOCITY OR POSITION
        :param apply_command: Defaults to true. apply the command immediately
        :return:
        """
        joint_idx = None
        if isinstance(joint_name_or_idx, str):
            joint_idx = self.get_joint_idx_from_name(joint_name_or_idx)
        else:
            joint_idx = joint_name_or_idx

        if self.is_joint_idx_valid(joint_idx):
            self._cmd.name = self._joints_temp.name
            self._cmd.velocity = self._joints_temp.velocity
            self._cmd.effort = self._joints_temp.effort
            # Acquire new joint positions
            position_list = list(self._joints_temp.position)
            position_list[joint_idx] = cmd
            self._cmd.position = tuple(position_list)
            # Reset Temp Joints
            self._joints_temp = self._cmd
            #print(cmd)
            if apply_command:
                self._apply_command()

    def set_joint_pos(self, joint_name_or_idx, p):
        """
        Set the joint position based on the index or names. Check the get_joint_names to see the list of
        joint names for indexes
        :param joint_name_or_idx:
        :param p:
        :return:
        """
        mag_p = p

        if joint_name_or_idx == 7:
            # Grippers in Isaac Sim Goes Opposite Way
            mag_p = -p
        if joint_name_or_idx == 3:
            # Yaw Rotations
            mag_p = self.getShortestAngle(joint_name_or_idx, p)
        self._set_joint_command(joint_name_or_idx, mag_p, "Position")

    def getShortestAngle(self, joint_idx, p):
        position_list = list(self._joints.position)
        current = position_list[joint_idx]
        if current < 0.0:
            current = 2 * math.pi + current
        delta_angle = math.fmod(p - current + 3 * math.pi, 2 * math.pi) - math.pi
        return delta_angle