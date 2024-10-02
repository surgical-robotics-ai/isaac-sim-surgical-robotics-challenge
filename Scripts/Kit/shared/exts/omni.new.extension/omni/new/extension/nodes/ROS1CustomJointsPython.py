"""
This is the implementation of the OGN node defined in ROS1CustomJointsPython.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy

import rospy
import threading
from std_msgs.msg import Float32

class OgnROS1CustomNodeInternal():
    def __init__(self):
        self.initialized = False
        self.data = []
        self._pub_thread = []

    def create_publisher(self, topicName):
        rospy.init_node(topicName, anonymous=True)
        self.rate = rospy.Rate(10)
        self.topicName = topicName
        self.publisher = rospy.Publisher(name=self.topicName, data_class=Float32, tcp_nodelay=True, queue_size=10)
        self.initialized = True
    
    def run_publisher(self):
        self._pub_thread = threading.Thread(target=self.publish)
        self._pub_thread.daemon = True
        self._pub_thread.start()

    def publish(self):
        while not rospy.is_shutdown():
            cmd = Float32()
            cmd.data = self.data
            self.publisher.publish(cmd)
            self.rate.sleep()

    def update(self, new_data):
        self.data = new_data

class ROS1CustomJointsPython:
    """
         ROS1 Custom Joints Python
    """

    @staticmethod
    def internal_state():
        try:
            rospy.init_node("topicName", anonymous=True)
        except:
            pass
        return OgnROS1CustomNodeInternal()

    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        try:
            # With the compute in a try block you can fail the compute by raising an exception
            state = db.internal_state
            if (not state.initialized):
                state.create_publisher(db.inputs.topicName)
                state.run_publisher()
            state.update(db.inputs.joint)

        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True
