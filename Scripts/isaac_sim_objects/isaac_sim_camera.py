from .isaac_sim_base_object import BaseObject
from transformations import quaternion_from_euler, euler_from_quaternion

# Isaac Sim based conversion of AMBF Camera for use in simulator / rostopic communication.
class Camera(BaseObject):
    def __init__(self, a_name, time_out=0.1):
        """
        Constructor
        :param a_name:
        """
        super(Camera, self).__init__(a_name, time_out)  # Set duration of Watchdog expiry
        self.object_type = "CAMERA"
        self.body_type = "KINEMATIC"

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
        """
        class pos_conversion:
            def __init__(self, vals):
                self.x = vals.x / 10
                self.y = vals.y / 10
                self.z = vals.z / 10
        return pos_conversion(self._state.pose.pose.position)
        """
        return self._state.pose.pose.position
    
    def ros_cb(self, data):
        """
        Call function for ROS topics
        :param data:
        :return:
        """
        self._state = data