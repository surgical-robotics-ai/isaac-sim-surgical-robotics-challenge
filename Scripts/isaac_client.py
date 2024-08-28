import rospy
from std_msgs.msg import Empty
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from .isaac_sim_objects.isaac_sim_camera import Camera
from .isaac_sim_objects.isaac_sim_psm import PSM

"""
# IsaacClient
# IsaacClient is a script modeled off of ambf_client.py to act as a intermediary between external
# controllers and Isaac Sim, utilizing rostopics to control different poses and actions of objects
# within the simulator
# Like the AMBF version, it creates wrapper objects around rostopics for the user to comminucate with
# to control models within the simulator

# Version 0.0.1
Currently supports communication with Isaac Sim Camera and PSM Object
"""
class IsaacClient:
    def __init__(self, client_name='isaac_client'):
        self._ros_topics = []
        self._sub_list = []
        self._objects_dict = {}
        self._sub_thread = []
        self._pub_thread = []
        self._world_name = ''
        self._common_obj_namespace = ''
        self._client_name = client_name
        self._world_handle = None
        self._rate = None
        pass

    def set_publish_rate(self, rate):
        self._rate = rospy.Rate(rate)

    def create_objs_from_rostopics(self, publish_rate):
        # Check if a node is running, if not create one
        # else get the name of the node
        if "/unnamed" == rospy.get_name():
            rospy.init_node(self._client_name)
        else:
            self._client_name = rospy.get_name()
        self.set_publish_rate(publish_rate)
        rospy.on_shutdown(self.clean_up)
        self._ros_topics = rospy.get_published_topics()

        # Create Objects for the Isaac Sim Rostopics
        for i in range(len(self._ros_topics)):
            topic_name = self._ros_topics[i][0]
            msg_type = self._ros_topics[i][1]
            if topic_name == '/CameraFrame':
                base_obj = Camera(topic_name)
                base_obj._state = Odometry()
                base_obj._cmd = Odometry()
                base_obj._sub = rospy.Subscriber(topic_name, Odometry, base_obj.ros_cb)
                base_obj._pub = rospy.Publisher(name='/CameraFramePublisher', data_class=Odometry,
                                                tcp_nodelay=True, queue_size=10)
                self._objects_dict[base_obj.get_name()] = base_obj
            if topic_name == '/psm1/baselink':
                base_obj = PSM(topic_name)
                base_obj._state = Odometry()
                base_obj._joints = JointState()
                base_obj._cmd = JointState()
                base_obj._subj = rospy.Subscriber('/PSM1/measured_js', JointState, base_obj.ros_j_cb)
                base_obj._sub = rospy.Subscriber('/psm1/baselink', Odometry, base_obj.ros_cb)
                base_obj._pub = rospy.Publisher(name='/PSM1/move_jp', data_class=JointState, 
                                                tcp_nodelay=True, queue_size=10)
                self._objects_dict[base_obj.get_name()] = base_obj
            if topic_name == '/psm2/baselink':
                base_obj = PSM(topic_name)
                base_obj._state = Odometry()
                base_obj._joints = JointState()
                base_obj._cmd = JointState()
                base_obj._subj = rospy.Subscriber('/PSM2/measured_js', JointState, base_obj.ros_j_cb)
                base_obj._sub = rospy.Subscriber('/psm2/baselink', Odometry, base_obj.ros_cb)
                base_obj._pub = rospy.Publisher(name='/PSM2/move_jp', data_class=JointState, 
                                                tcp_nodelay=True, queue_size=10)
                self._objects_dict[base_obj.get_name()] = base_obj

    def connect(self, default_publish_rate=120):
        self.create_objs_from_rostopics(default_publish_rate)
        self.start()

    def refresh(self):
        self.clean_up()
        self.connect()

    def start(self):
        self._start_pubs()

    def get_common_namespace(self):
        return self._common_obj_namespace

    def get_world_handle(self):
        return self._world_handle

    def get_obj_names(self):
        obj_names = []
        for key, obj in self._objects_dict.items():
            obj_names.append(obj.get_name())
        return obj_names
    
    def get_obj_handle(self, a_name):
        found_obj = None
        obj = self._objects_dict.get(a_name)
        if obj:
            found_obj = obj
        else:
            # Try matching the object name to existing names with the closest match
            objects = []
            for key, item in self._objects_dict.items():
                if key.find(a_name) >= 0:
                    objects.append(item)

            if len(objects) == 1:
                found_obj = objects[0]
            elif len(objects) == 0:
                print(a_name, 'NAMED OBJECT NOT FOUND')
                found_obj = None
            elif len(objects) > 1:
                print('WARNING FOUND ', len(objects), 'WITH MATCHING NAME:')
                for i in range(len(objects)):
                    print(objects[i].get_name())
                print('PLEASE SPECIFY FULL NAME TO GET THE OBJECT HANDLE')
                found_obj = None

        if type(found_obj) == Camera:
            found_obj.set_active()
        elif type(found_obj) == PSM:
            found_obj.set_active()

        return found_obj

    def get_obj_pose(self, a_name):
        obj = self._objects_dict.get(a_name)
        if obj is not None:
            return obj.pose
        else:
            return None

    def set_obj_cmd(self, a_name, fx, fy, fz, nx, ny, nz):
        obj = self._objects_dict.get(a_name)
        obj.command(fx, fy, fz, nx, ny, nz)

    def _start_pubs(self):
        self._pub_thread = threading.Thread(target=self._run_obj_publishers)
        self._pub_thread.daemon = True
        self._pub_thread.start()

    def _run_obj_publishers(self):
        while not rospy.is_shutdown():
            for key, obj in self._objects_dict.items():
                if obj.is_active():
                    obj.run_publisher()
            self._rate.sleep()

    def print_active_topics(self):
        print(self._ros_topics)
        pass

    def print_summary(self):
        print('_________________________________________________________')
        print('---------------------------------------------------------')
        print('CLIENT FOR CREATING OBJECTS FROM ROSTOPICS')
        print('Searching Object names from ros topics with')
        print('Prefix: ', self._search_prefix_str)
        print('Suffix: ', self._search_suffix_str)
        print('Number of OBJECTS found', len(self._objects_dict))
        for key, value in self._objects_dict.items():
            print(key)
        print('---------------------------------------------------------')

    def clean_up(self):
        for key, val in self._objects_dict.items():
            val.pub_flag = False
            print('Closing publisher for: ', key)
        self._objects_dict.clear()