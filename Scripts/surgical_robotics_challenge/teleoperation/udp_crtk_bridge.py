#!/usr/bin/env python3
"""
udp_crtk_bridge.py

A UDP <-> CRTK-ROS bridge that makes IsaacSim look like a dVRK `sawSocketStreamer`
to the Meta Quest teleoperation app. The Quest cannot tell whether it is talking
to a real robot's sawSocketStreamer or to this bridge: same wire format, same
verbs, same frames.

Wire protocol (reverse-engineered from the Quest's UDPComm.cs / MovementController):
  Bridge SENDS  (bridge -> Quest, what the Quest parses):
    PSM pose : {"setpoint_cp": {"Position": {"Rotation": [[3x3]], "Translation":[x,y,z]}, "Timestamp":t, "Valid":true, "AutomaticTimestamp":true}}
    PSM jaw  : {"jaw/setpoint_js": {"Name":["jaw"], "Position":[angle], "Effort":[0], "Velocity":[0], "Timestamp":t, "Valid":true, "AutomaticTimestamp":true}}
    ECM pose : {"setpoint_cp": {...}}                       (same cp shape)
    ECM joints:{"setpoint_js": {"Name":["yaw","pitch","insertion","roll"], "Position":[4], ...}}
  Bridge RECEIVES (Quest -> bridge):
    PSM pose : {"move_cp":{"Goal":{"Rotation":[[3x3]],"Translation":[x,y,z]}}}
    PSM jaw  : {"jaw/move_jp":{"Goal":[angle]}}
    ECM joints:{"move_jp":{"Goal":[yaw,pitch,insertion,roll]}}

Frames / units: PSM cp is tool-tip-in-PSM-base-frame, SI meters (PSM_arm.measured_cp
returns FK in base frame; servo_cp runs IK on a tip-in-base frame). The bridge does
NOT swap axes or rescale -- it passes base-frame poses straight through, exactly as a
real sawSocketStreamer would. Any constant base-frame offset is handled by your
external calibration. If orientation comes out mirrored, flip TRANSPOSE_ROTATION_OUT.

Ports (from UDPComm.cs): the Quest binds its receive sockets on 48051 (PSM1),
48052 (PSM2), 48053 (ECM), and its senders use 5701/5702/5703. So the bridge sends
state to QUEST_IP:4805x and binds 570x to receive commands (matching the real-robot
convention so the Quest's pre-set remote endpoints line up). The Quest will not send
anything until it has received PSM setpoints, so the bridge must know QUEST_IP up
front (no "reply to sender" -- that would deadlock).

ECM note: /CRTK/ecm/measured_js is hardcoded zeros (ecm_arm.py never updates
_measured_jp), so the bridge ECHOES the last commanded ECM joint vector back as
setpoint_js. This is also the hook for the future fixed-camera (no-ECM) setup.

Prerequisites (two terminals before this one):
  T1:  ./ambf_simulator --launch_file <srch>/launch.yaml -l 0,1,3,4,14,15 -p 120 -t 1
  T2:  python3 <srch>/scripts/surgical_robotics_challenge/launch_crtk_interface.py
"""

import argparse
import json
import math
import socket

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# ============================ CONFIG =========================================
QUEST_IP = "127.0.0.1"     # destination for outbound state (the headset's IP).
BIND_IP  = "0.0.0.0"       # local bind for the command-receive side.

# Per-arm: (port the bridge sends state FROM / receives commands ON, Quest's listen port)
PORTS = {
    "PSM1": {"bind": 5701, "quest": 48051},
    "PSM2": {"bind": 5702, "quest": 48052},
    "ecm":  {"bind": 5703, "quest": 48053},
}

SEND_HZ = 60.0             # outbound state rate
RECV_POLL_HZ = 200.0       # how often we drain inbound command datagrams

# If the Quest shows a mirrored/transposed orientation, set this True first.
TRANSPOSE_ROTATION_OUT = False

# CRTK ROS topics
NS = ""
PSM_MEASURED_CP = NS + "/{arm}/measured_cp"     # PoseStamped (tip-in-base)
PSM_SERVO_CP    = NS + "/{arm}/servo_cp"         # PoseStamped (we publish commands here)
PSM_JAW_SERVO   = NS + "/{arm}/jaw/servo_jp"     # JointState
ECM_MEASURED_CP = NS + "/ecm/measured_cp"        # PoseStamped (camera-in-world)
ECM_SERVO_JP    = NS + "/ecm/servo_jp"           # JointState

ECM_JOINT_NAMES = ["yaw", "pitch", "insertion", "roll"]
DEFAULT_JAW = 0.5          # matches PSM.__init__ set_jaw_angle(0.5)

# ---- Startup home pose (EXPERIMENT) -----------------------------------------
# To get the PSM tool tip OFF the near-singular zero-insertion configuration before
# any Quest command arrives, the bridge streams these hardcoded poses to servo_cp at
# startup and HOLDS them until the first real move_cp for that arm is received. After
# STARTUP_SETTLE_S the arm has settled here; once a Quest packet arrives, control hands
# over. Poses are tool-tip-in-PSM-base (captured from /CRTK/<arm>/measured_cp), so they
# feed servo_cp directly with no conversion. Set HOLD_HOME=False to send home once and
# immediately allow Quest control (the literal "command, wait, bridge").
HOLD_HOME = True
STARTUP_SETTLE_S = 1.0     # seconds to stream home before logging "ready" (arm settles)
HOME_POSE = {
    "PSM1": {
        "pos":  (0.0042284022113332174, 0.05971935451458767, -0.246578067229215),
        "quat": (0.7067932658533179, 0.7070301743729386, 0.021850683265487385, -0.008611591688199863),
    },
    "PSM2": {
        "pos":  (0.0042284022113332174, 0.05971935451458767, -0.246578067229215),
        "quat": (0.7067932658533179, 0.7070301743729386, 0.021850683265487385, -0.008611591688199863),
    },
}
# =============================================================================


# ---- rotation <-> quaternion helpers ----------------------------------------
def quat_to_matrix(qx, qy, qz, qw):
    """Unit quaternion (x,y,z,w) -> 3x3 rotation matrix (numpy)."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)],
    ])


def matrix_to_quat(R):
    """3x3 rotation matrix -> quaternion (x,y,z,w), Shepperd's method."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


# ---- JSON payload builders (match the sawSocketStreamer shape) --------------
def cp_payload(key, R, translation, stamp):
    # ORDER MATTERS: Rotation must come before Translation. The Quest's
    # JsonHelper.GetRotation stops at the first '"' after "Rotation":, so it relies
    # on Translation's key-quote to terminate the rotation array. Reordering breaks it.
    if TRANSPOSE_ROTATION_OUT:
        R = np.asarray(R).T
    return {
        key: {
            "AutomaticTimestamp": True,
            "Position": {
                "Rotation": [[float(R[i][j]) for j in range(3)] for i in range(3)],
                "Translation": [float(v) for v in translation],
            },
            "Timestamp": float(stamp),
            "Valid": True,
        }
    }


def js_payload(key, names, positions, stamp):
    n = len(positions)
    return {
        key: {
            "AutomaticTimestamp": True,
            "Effort": [0.0] * n,
            "Name": list(names),
            "Position": [float(v) for v in positions],
            "Timestamp": float(stamp),
            "Valid": True,
            "Velocity": [0.0] * n,
        }
    }


def stamp_to_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


# ---- one bidirectional UDP socket bound per arm -----------------------------
class ArmSocket:
    def __init__(self, node, name, bind_port, quest_port):
        self.node = node
        self.name = name
        self.dest = (QUEST_IP, quest_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((BIND_IP, bind_port))
        self.sock.setblocking(False)

    def send_json(self, obj):
        try:
            # COMPACT separators are REQUIRED: the Quest's JsonHelper.JawAngle/Joints
            # parsers use index math that assumes "Position":[...] with no space after
            # the colon (matching real dVRK sawSocketStreamer output). A default
            # ", "/": " from json.dumps shifts their window and breaks float.Parse.
            data = json.dumps(obj, separators=(",", ":")).encode("utf-8")
            self.sock.sendto(data, self.dest)
        except OSError as e:
            self.node.get_logger().error(f"[{self.name}] send failed: {e}")

    def drain(self, handler):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except BlockingIOError:
                return
            except OSError as e:
                self.node.get_logger().error(f"[{self.name}] recv error: {e}")
                return
            text = data.decode("utf-8", errors="ignore").strip().strip("\x00")
            if not text:
                continue
            try:
                obj = json.loads(text)
            except json.JSONDecodeError as e:
                self.node.get_logger().warn(f"[{self.name}] bad JSON: {e}")
                continue
            handler(obj)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# ---- PSM: send setpoint_cp + jaw/setpoint_js; recv move_cp + jaw/move_jp -----
class PsmBridge:
    def __init__(self, node, arm):
        self.node = node
        self.arm = arm
        self.io = ArmSocket(node, arm, PORTS[arm]["bind"], PORTS[arm]["quest"])

        self.latest_cp = None          # (R 3x3, translation, stamp)
        self.last_jaw = DEFAULT_JAW

        # Startup home / hold state
        self.home = HOME_POSE.get(arm)         # None if no home defined for this arm
        self.holding_home = self.home is not None
        self._home_msg = self._build_home_msg() if self.home is not None else None
        self._first_cmd_seen = False

        node.create_subscription(
            PoseStamped, PSM_MEASURED_CP.format(arm=arm), self._on_measured_cp, 10
        )
        self.servo_cp_pub = node.create_publisher(PoseStamped, PSM_SERVO_CP.format(arm=arm), 10)
        self.jaw_pub = node.create_publisher(JointState, PSM_JAW_SERVO.format(arm=arm), 10)
        node.get_logger().info(
            f"[{arm}] send->{self.io.dest}  recv:{PORTS[arm]['bind']}  "
            f"(cp from {PSM_MEASURED_CP.format(arm=arm)}, cmd to {PSM_SERVO_CP.format(arm=arm)})"
        )
        if self.holding_home:
            node.get_logger().info(
                f"[{arm}] holding hardcoded home pos={self.home['pos']} "
                f"until first Quest command (HOLD_HOME={HOLD_HOME})."
            )

    def _build_home_msg(self):
        px, py, pz = self.home["pos"]
        qx, qy, qz, qw = self.home["quat"]
        m = PoseStamped()
        m.header.frame_id = f"{self.arm}/baselink"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, pz
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = (
            qx, qy, qz, qw
        )
        return m

    def publish_home(self):
        """Publish the hardcoded home pose to servo_cp (re-stamped each call)."""
        if self._home_msg is None:
            return
        self._home_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.servo_cp_pub.publish(self._home_msg)

    def _on_measured_cp(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        R = quat_to_matrix(o.x, o.y, o.z, o.w)
        self.latest_cp = (R, (p.x, p.y, p.z), stamp_to_sec(msg))

    def on_send_tick(self, now_sec):
        if self.latest_cp is not None:
            R, t, stamp = self.latest_cp
            self.io.send_json(cp_payload("setpoint_cp", R, t, stamp))
        # jaw feedback: echo last commanded jaw (real interface doesn't publish jaw measured)
        self.io.send_json(js_payload("jaw/setpoint_js", ["jaw"], [self.last_jaw], now_sec))

    def poll(self):
        self.io.drain(self._handle)

    def _handle(self, obj):
        if "jaw/move_jp" in obj:
            goal = obj["jaw/move_jp"].get("Goal", [])
            if goal:
                self.last_jaw = float(goal[0])
                js = JointState()
                js.header.stamp = self.node.get_clock().now().to_msg()
                js.name = ["jaw"]
                js.position = [self.last_jaw]
                self.jaw_pub.publish(js)
        elif "move_cp" in obj or "servo_cp" in obj:
            key = "move_cp" if "move_cp" in obj else "servo_cp"
            goal = obj[key].get("Goal", {})
            R = goal.get("Rotation")
            T = goal.get("Translation")
            if R is None or T is None:
                return

            # First valid Quest pose command releases the startup home hold.
            if not self._first_cmd_seen:
                self._first_cmd_seen = True
                if self.holding_home:
                    self.holding_home = False
                    self.node.get_logger().info(
                        f"[{self.arm}] first Quest command received -> releasing home hold, "
                        f"Quest now in control."
                    )
            # While still holding home (HOLD_HOME and no command yet), ignore Quest poses
            # so the arm stays parked at the hardcoded home. (Once released, always pass.)
            if HOLD_HOME and self.holding_home:
                return

            qx, qy, qz, qw = matrix_to_quat(R)
            cmd = PoseStamped()
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd.header.frame_id = f"{self.arm}/baselink"
            cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z = (
                float(T[0]), float(T[1]), float(T[2])
            )
            cmd.pose.orientation.x = qx
            cmd.pose.orientation.y = qy
            cmd.pose.orientation.z = qz
            cmd.pose.orientation.w = qw
            # NOTE: we publish to servo_cp (immediate IK) even though the Quest labels
            # it move_cp -- move_cp spawns a trajectory thread per call, which is wrong
            # for a 60 Hz stream. servo_cp is the correct streaming target.
            self.servo_cp_pub.publish(cmd)

    def close(self):
        self.io.close()


# ---- ECM: send setpoint_cp + setpoint_js(echo); recv move_jp -> servo_jp -----
class EcmBridge:
    def __init__(self, node):
        self.node = node
        self.io = ArmSocket(node, "ecm", PORTS["ecm"]["bind"], PORTS["ecm"]["quest"])
        self.latest_cp = None
        self.last_joints = [0.0, 0.0, 0.0, 0.0]   # echoed back as setpoint_js

        node.create_subscription(PoseStamped, ECM_MEASURED_CP, self._on_measured_cp, 10)
        self.servo_jp_pub = node.create_publisher(JointState, ECM_SERVO_JP, 10)
        node.get_logger().info(
            f"[ecm] send->{self.io.dest}  recv:{PORTS['ecm']['bind']}  "
            f"(cp from {ECM_MEASURED_CP}, joints echoed; cmd to {ECM_SERVO_JP})"
        )

    def _on_measured_cp(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        R = quat_to_matrix(o.x, o.y, o.z, o.w)
        self.latest_cp = (R, (p.x, p.y, p.z), stamp_to_sec(msg))

    def on_send_tick(self, now_sec):
        if self.latest_cp is not None:
            R, t, stamp = self.latest_cp
            self.io.send_json(cp_payload("setpoint_cp", R, t, stamp))
        # ECM joint feedback is echoed (AMBF measured_js is hardcoded zeros).
        self.io.send_json(js_payload("setpoint_js", ECM_JOINT_NAMES, self.last_joints, now_sec))

    def poll(self):
        self.io.drain(self._handle)

    def _handle(self, obj):
        if "move_jp" in obj or "servo_jp" in obj:
            key = "move_jp" if "move_jp" in obj else "servo_jp"
            goal = obj[key].get("Goal", [])
            if len(goal) >= 4:
                self.last_joints = [float(v) for v in goal[:4]]
                js = JointState()
                js.header.stamp = self.node.get_clock().now().to_msg()
                js.name = list(ECM_JOINT_NAMES)
                js.position = list(self.last_joints)
                # map move_jp -> servo_jp (ECM interface only subscribes servo_jp)
                self.servo_jp_pub.publish(js)

    def close(self):
        self.io.close()


class UdpCrtkBridge(Node):
    def __init__(self):
        super().__init__("udp_crtk_bridge")
        self.bridges = [PsmBridge(self, "PSM1"), PsmBridge(self, "PSM2"), EcmBridge(self)]
        self.PSM_bridges = [b for b in self.bridges if isinstance(b, PsmBridge)]
        self._start_sec = self.get_clock().now().nanoseconds * 1e-9
        self._settle_logged = False
        self.create_timer(1.0 / SEND_HZ, self._on_send)
        self.create_timer(1.0 / RECV_POLL_HZ, self._on_recv)
        self.get_logger().info(f"UDP-CRTK bridge up. Sending state to {QUEST_IP}.")

    def _on_send(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Startup home: stream the hardcoded home to servo_cp.
        #  - HOLD_HOME=True : keep publishing home until that arm gets its first Quest cmd.
        #  - HOLD_HOME=False: publish home only during the settle window, then stop and let
        #                     Quest commands through (the literal "command, wait, bridge").
        for b in self.PSM_bridges:
            if not b.holding_home:
                continue
            if HOLD_HOME:
                b.publish_home()
            else:
                if now - self._start_sec < STARTUP_SETTLE_S:
                    b.publish_home()
                else:
                    b.holding_home = False  # settle window over; Quest now in control

        if (not self._settle_logged) and (now - self._start_sec >= STARTUP_SETTLE_S):
            self._settle_logged = True
            self.get_logger().info(
                f"Startup settle window ({STARTUP_SETTLE_S:.1f}s) elapsed; "
                f"PSMs parked at home. "
                + ("Holding until first Quest command." if HOLD_HOME
                   else "Quest commands now active.")
            )

        for b in self.bridges:
            b.on_send_tick(now)

    def _on_recv(self):
        for b in self.bridges:
            b.poll()

    def close_all(self):
        for b in self.bridges:
            b.close()


def main():
    global QUEST_IP
    parser = argparse.ArgumentParser()
    parser.add_argument("--quest-ip", default=QUEST_IP, help="Headset IP to send state to")
    args = parser.parse_args()
    QUEST_IP = args.quest_ip

    rclpy.init()
    node = UdpCrtkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_all()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
