#!/usr/bin/env python
# //==============================================================================
# /*
#     Software License Agreement (BSD License)
#     Copyright (c) 2020-2021 Johns Hopkins University (JHU), Worcester Polytechnic Institute (WPI) All Rights Reserved.


#     All rights reserved.

#     Redistribution and use in source and binary forms, with or without
#     modification, are permitted provided that the following conditions
#     are met:

#     * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.

#     * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.

#     * Neither the name of authors nor the names of its contributors may
#     be used to endorse or promote products derived from this software
#     without specific prior written permission.

#     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#     "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#     LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#     FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#     COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#     INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#     BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#     CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#     LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#     ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#     POSSIBILITY OF SUCH DAMAGE.


#     \author    <amunawar@jhu.edu>
#     \author    Adnan Munawar
#     \version   1.0
# */
# //==============================================================================

import PyKDL
from PyKDL import Frame, Rotation, Vector
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, TwistStamped, WrenchStamped, Wrench
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Bool
import rospy, rostopic
import time
import numpy as np


# Utilities
def kdl_frame_to_pose_stamped_msg(kdl_pose):
    ps = PoseStamped()
    p = ps.pose
    p.position.x = kdl_pose.p[0]
    p.position.y = kdl_pose.p[1]
    p.position.z = kdl_pose.p[2]

    p.orientation.x = kdl_pose.M.GetQuaternion()[0]
    p.orientation.y = kdl_pose.M.GetQuaternion()[1]
    p.orientation.z = kdl_pose.M.GetQuaternion()[2]
    p.orientation.w = kdl_pose.M.GetQuaternion()[3]

    return ps


def kdl_frame_to_transform_stamped_msg(kdl_pose):
    ps = TransformStamped()
    p = ps.transform
    p.translation.x = kdl_pose.p[0]
    p.translation.y = kdl_pose.p[1]
    p.translation.z = kdl_pose.p[2]

    p.rotation.x = kdl_pose.M.GetQuaternion()[0]
    p.rotation.y = kdl_pose.M.GetQuaternion()[1]
    p.rotation.z = kdl_pose.M.GetQuaternion()[2]
    p.rotation.w = kdl_pose.M.GetQuaternion()[3]

    return ps


def pose_stamped_to_transform_stamped(ps):
    ts = TransformStamped()
    ts.header = ps.header
    ts.transform.translation.x = ps.pose.position.x
    ts.transform.translation.y = ps.pose.position.y
    ts.transform.translation.z = ps.pose.position.z

    ts.transform.rotation.x = ps.pose.orientation.x
    ts.transform.rotation.y = ps.pose.orientation.y
    ts.transform.rotation.z = ps.pose.orientation.z
    ts.transform.rotation.w = ps.pose.orientation.w

    return ts


def kdl_wrench_to_wrench_msg(kdl_wrench):
    ws = WrenchStamped()
    w = ws.wrench
    w.force.x = kdl_wrench.force[0]
    w.force.y = kdl_wrench.force[1]
    w.force.z = kdl_wrench.force[2]

    w.torque.x = kdl_wrench.torque[0]
    w.torque.y = kdl_wrench.torque[1]
    w.torque.z = kdl_wrench.torque[2]

    return ws


def pose_msg_to_kdl_frame(msg_pose):
    pose = msg_pose.pose
    f = Frame()
    f.p[0] = pose.position.x
    f.p[1] = pose.position.y
    f.p[2] = pose.position.z
    f.M = Rotation.Quaternion(pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z,
                              pose.orientation.w)

    return f

def transform_msg_to_kdl_frame(msg_pose):
    pose = msg_pose.transform
    f = Frame()
    f.p[0] = pose.translation.x
    f.p[1] = pose.translation.y
    f.p[2] = pose.translation.z
    f.M = Rotation.Quaternion(pose.rotation.x,
                              pose.rotation.y,
                              pose.rotation.z,
                              pose.rotation.w)

    return f


def vector_to_effort_msg(effort):
    msg = JointState()
    msg.effort = effort
    return msg


def get_crtk_cp_msg_type_from_str(msg_type_str):
    if msg_type_str == 'geometry_msgs/PoseStamped':
        return PoseStamped
    elif msg_type_str == 'geometry_msgs/TransformStamped':
        return TransformStamped
    else:
        print("Exception! Message Type: %s CRTK CP MESSAGE TYPE IS NEITHER PoseStamped or TransformStamped", msg_type_str)
        raise TypeError


# Init everything related to Geomagic
class MTM:
    # The name should include the full qualified prefix. I.e. '/Geomagic/', or '/omniR_' etc.
    def __init__(self, name):
        pose_sub_topic_name = name + 'measured_cp'
        twist_topic_name = name + 'measured_cv'
        joint_state_sub_topic_name = name + 'measured_js'
        gripper_topic_name = name + 'gripper/measured_js'
        clutch_topic_name = '/footpedals/clutch'
        coag_topic_name = '/console/operator_present'

        pose_pub_topic_name = name + 'servo_cp'
        wrench_pub_topic_name = name + 'body/servo_cf'
        effort_pub_topic_name = name + 'servo_jf'
        grav_comp_topic_name = name + 'use_gravity_compensation'

        self.cur_pos_msg = None
        self.pre_coag_pose_msg = None

        self._active = False
        self._scale = 1.0
        self.pose = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.twist = PyKDL.Twist()
        R_off = Rotation.RPY(0, 0, 0)
        self._T_baseoffset = Frame(R_off, Vector(0, 0, 0))
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        self._T_tipoffset = Frame(Rotation().RPY(0, 0, 0), Vector(0, 0, 0))
        self.clutch_button_pressed = False  # Used as Position Engage Clutch
        self.coag_button_pressed = False  # Used as Position Engage Coag
        self.gripper_angle = 0

        self.switch_psm = False
        self._button_msg_time = rospy.Time.now()
        self._switch_psm_duration = rospy.Duration(0.5)

        self._arm_publishing = False

        self._jp = []
        self._jv = []
        self._jf = []

        print(rostopic.get_topic_type(pose_sub_topic_name))
        print(rostopic.get_topic_type(pose_pub_topic_name))
        self.MEASURED_CP_MESSAGE_TYPE = get_crtk_cp_msg_type_from_str(rostopic.get_topic_type(pose_sub_topic_name)[0])
        self.SERVO_CP_MESSAGE_TYPE = get_crtk_cp_msg_type_from_str(rostopic.get_topic_type(pose_pub_topic_name)[0])

        self._pose_sub = rospy.Subscriber(
            pose_sub_topic_name, self.MEASURED_CP_MESSAGE_TYPE, self.pose_cb, queue_size=1)
        self._state_sub = rospy.Subscriber(
            joint_state_sub_topic_name, JointState, self.state_cb, queue_size=1)
        self._gripper_sub = rospy.Subscriber(
            gripper_topic_name, JointState, self.gripper_cb, queue_size=1)
        self._twist_sub = rospy.Subscriber(
            twist_topic_name, TwistStamped, self.twist_cb, queue_size=1)
        self._clutch_button_sub = rospy.Subscriber(
            clutch_topic_name, Joy, self.clutch_buttons_cb, queue_size=1)
        self._coag_button_sub = rospy.Subscriber(
            coag_topic_name, Joy, self.coag_buttons_cb, queue_size=1)

        self._pos_pub = rospy.Publisher(
            pose_pub_topic_name, self.SERVO_CP_MESSAGE_TYPE, queue_size=1)
        self._wrench_pub = rospy.Publisher(
            wrench_pub_topic_name, WrenchStamped, queue_size=1)
        self._effort_pub = rospy.Publisher(
            effort_pub_topic_name, JointState, queue_size=1)
        self._gravity_comp_pub = rospy.Publisher(
            grav_comp_topic_name, Bool, queue_size=1)

        print('Creating MTM Device Named: ', name, ' From ROS Topics')
        self._msg_counter = 0

    def set_base_frame(self, frame):
        self._T_baseoffset = frame
        self._T_baseoffset_inverse = self._T_baseoffset.Inverse()
        pass

    def optimize_wrist_platform(self):
        if self._arm_publishing is False:
            return

        qs = self._jp
        vs = self._jv
        Kp_4 = 0.3
        Kd_4 = 0.03
        lim_4 = 0.2

        Kp_6 = 0.0
        Kd_6 = 0.0
        lim_6 = 0.01

        # Limits for Wrist Pitch (Joint 5)
        a_lim_5 = -1.5
        b_lim_5 = 1.2
        c_lim_5 = 1.8

        sign = 1
        if a_lim_5 < qs[4] <= b_lim_5:
            sign = 1
        elif b_lim_5 < qs[4] < c_lim_5:
            range = c_lim_5 - b_lim_5
            normalized_val = (qs[4] - b_lim_5) / range
            centerd_val = normalized_val - 0.5
            sign = -centerd_val * 2
            print('MID VAL:', sign)
            # sign = 0
        else:
            sign = -1

        e = qs[5]
        tau_4 = Kp_4 * e * sign - Kd_4 * vs[3]
        tau_4 = np.clip(tau_4, -lim_4, lim_4)

        tau_6 = -Kp_6 * qs[5] - Kd_6 * vs[5]
        tau_6 = np.clip(tau_6, -lim_6, lim_6)

        js_cmd = [0.0]*7
        js_cmd[3] = tau_4
        js_cmd[5] = tau_6
        self.servo_jf(js_cmd)

    def set_tip_frame(self, frame):
        self._T_tipoffset = frame
        pass

    def set_scale(self, scale):
        self._scale = scale

    def get_scale(self):
        return self._scale

    def pose_cb(self, msg):
        self.cur_pos_msg = msg
        if self.pre_coag_pose_msg is None:
            self.pre_coag_pose_msg = self.cur_pos_msg
        if type(msg) == PoseStamped:
            cur_frame = pose_msg_to_kdl_frame(msg)
        elif type(msg) == TransformStamped:
            cur_frame = transform_msg_to_kdl_frame(msg)
        cur_frame.p = cur_frame.p * self._scale
        self.pose = self._T_baseoffset_inverse * cur_frame * self._T_tipoffset
        # Mark active as soon as first message comes through
        self._active = True
        pass

    def state_cb(self, msg):
        self._jp = msg.position
        self._jv = msg.velocity
        self._jf = msg.effort
        self._arm_publishing = True

    def is_active(self):
        return self._active

    def gripper_cb(self, msg):
        min = 0.10
        max = 0.51
        self.gripper_angle = msg.position[0] + min / (max - min)
        pass

    def twist_cb(self, msg):
        twist = PyKDL.Twist()
        omega = msg.twist
        twist[0] = omega.linear.x
        twist[1] = omega.linear.y
        twist[2] = omega.linear.z
        twist[3] = omega.angular.x
        twist[4] = omega.angular.y
        twist[5] = omega.angular.z
        self.twist = self._T_baseoffset_inverse * twist
        pass

    def clutch_buttons_cb(self, msg):
        self.clutch_button_pressed = msg.buttons[0]
        self.pre_coag_pose_msg = self.cur_pos_msg
        if self.clutch_button_pressed:
            time_diff = rospy.Time.now() - self._button_msg_time
            if time_diff.to_sec() < self._switch_psm_duration.to_sec():
                print('Allow PSM Switch')
                self.switch_psm = True
            self._button_msg_time = rospy.Time.now()

    def coag_buttons_cb(self, msg):
        self.coag_button_pressed = msg.buttons[0]
        self.pre_coag_pose_msg = self.cur_pos_msg

    def command_force(self, force):
        pass

    def servo_cp(self, pose):
        if self.SERVO_CP_MESSAGE_TYPE == PoseStamped:
            if type(pose) == PyKDL.Frame:
                servo_cp_msg = kdl_frame_to_pose_stamped_msg(pose)
            elif type(pose) == PoseStamped:
                servo_cp_msg = pose
            elif type(pose) == TransformStamped:
                servo_cp_msg = pose
            else:
                raise TypeError
        elif self.SERVO_CP_MESSAGE_TYPE == TransformStamped:
            if type(pose) == PyKDL.Frame:
                servo_cp_msg = kdl_frame_to_transform_stamped_msg(pose)
            elif type(pose) == PoseStamped:
                servo_cp_msg = pose_stamped_to_transform_stamped(pose)
            elif type(pose) == TransformStamped:
                servo_cp_msg = pose
            else:
                raise TypeError
        else:
            print(self.SERVO_CP_MESSAGE_TYPE)
            raise TypeError

        self._pos_pub.publish(servo_cp_msg)

    def servo_cf(self, wrench):
        wrench = self._T_baseoffset_inverse * wrench
        wrench_msg = kdl_wrench_to_wrench_msg(wrench)
        self._wrench_pub.publish(wrench_msg)

    def servo_jf(self, torques):
        effort_msg = vector_to_effort_msg(torques)
        self._effort_pub.publish(effort_msg)

    def measured_cp(self):
        return self.pose

    def measured_jp(self):
        return self._jp

    def measured_jf(self):
        return self._jf

    def measured_cv(self):
        return self.twist

    def get_jaw_angle(self):
        return self.gripper_angle

    def enable_gravity_comp(self):
        self._gravity_comp_pub.publish(True)

    def disable_gravity_comp(self):
        self._gravity_comp_pub.publish(False)


def test():
    rospy.init_node('test_mtm')

    d = MTM('/dvrk/MTMR/')
    d.set_base_frame(Frame(Rotation.RPY(np.pi/2, 0, 0), Vector()))
    # rot_offset = Rotation.RPY(np.pi, np.pi/2, 0).Inverse()
    # tip_offset = Frame(rot_offset, Vector(0, 0, 0))
    # d.set_tip_frame(tip_offset)
    err_last = 0.0
    while not rospy.is_shutdown():
        if d.coag_button_pressed:
            d.optimize_wrist_platform()
        else:
            if d.is_active():
                d.servo_cp(d.pre_coag_pose_msg)

        # [r, p, y] = d.measured_cp().M.GetRPY()
        # f = 180.0 / 3.1404
        # r = round(r * f, 2)
        # p = round(p * f, 2)
        # y = round(y * f, 2)
        # print('Roll: ', r, ', Pitch: ', p, ', Yaw: ', y)
        time.sleep(0.05)


if __name__ == '__main__':
    test()
