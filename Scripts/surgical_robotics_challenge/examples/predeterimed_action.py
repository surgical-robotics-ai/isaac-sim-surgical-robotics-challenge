from surgical_robotics_challenge.kinematics.psmKinematics import *
from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM
import time
import rospy
from PyKDL import Frame, Rotation, Vector
from argparse import ArgumentParser
from surgical_robotics_challenge.utils.obj_control_gui import ObjectGUI
from surgical_robotics_challenge.utils.jnt_control_gui import JointGUI
from surgical_robotics_challenge.utils import interpolation
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils.utilities import get_boolean_from_opt
import numpy as np

# Determined Path 0, replaces gui object class with predetermined paths for the PSM to follow
class DP0:
    def __init__(self, obj_name, initial_xyz=[0, 0, 0], initial_rpy=[0, 0, 0], initial_gr=[0], range_xyz=1.0, range_rpy=3.14, resolution=0.0001):
        # Get initial starting values
        self.initial_xyz = initial_xyz
        self.initial_rpy = initial_rpy
        self.initial_gr = initial_gr
        self.range_xyz = range_xyz
        self.range_rpy = range_rpy
        self.resolution = resolution

        self.x = self.initial_xyz[0]
        self.y = self.initial_xyz[1]
        self.z = self.initial_xyz[2]
        self.ro = self.initial_rpy[0]
        self.pi = self.initial_rpy[1]
        self.ya = self.initial_rpy[2]
        self.gr = [0.0]

        self.obj_name = obj_name
    
        # Handles for determined paths / trajectories
        self.x_traj = np.zeros((1, 1))
        self.y_traj = np.zeros((1, 1))
        self.z_traj = np.zeros((1, 1))

        self.ro_traj = np.zeros((1, 1))
        self.pi_traj = np.zeros((1, 1))
        self.ya_traj = np.zeros((1, 1))

        self.gr_traj = np.zeros((1, 1))

        self.interpolate = interpolation.Interpolation()

        # Overall Time Taken
        self.count = 1
        self.time = 0

        # Fill trajectories
        self.create_trajs()
    
    def create_trajs(self):
        # Define paths
        self.create_pos_traj(self.initial_xyz, [0.003358, -0.015672, -0.115672])
        self.create_ang_traj(self.initial_rpy, [3.10, 1, 1.4])
        self.create_grip_traj(self.initial_gr, [0.35])

        self.create_pos_traj(self.initial_xyz, [0.005597, -0.017910, -0.124627])
        self.create_ang_traj(self.initial_rpy, [2.826, 0.940, 1.794671])
        self.create_grip_traj(self.initial_gr, [0.35])

        self.create_pos_traj(self.initial_xyz, [0.005597, -0.017910, -0.124627])
        self.create_ang_traj(self.initial_rpy, [2.826, 0.940, 1.794671])
        self.create_grip_traj(self.initial_gr, [0.025])

        self.create_pos_traj(self.initial_xyz, [0.005597, 0.0, -0.124627])
        self.create_ang_traj(self.initial_rpy, [2.826, 0.940, 1.794671])
        self.create_grip_traj(self.initial_gr, [0.025])

        # Define total path time
        self.time = 200 * 4

    def create_pos_traj(self, start, end):
        start_np = np.array(start)
        end_np = np.array(end)

        # 0 Velocity and Acceleration constants
        d = np.zeros(start_np.shape)
        self.interpolate.compute_interpolation_params(start_np, end_np, d, d, d, d, 0, 1)
        time_array = np.arange(0, 1, 0.005)
        traj = self.interpolate.get_interpolated_x(time_array)

        # Build Trajectory
        self.x_traj = np.concatenate((self.x_traj, traj[0]), axis=1)
        self.y_traj = np.concatenate((self.y_traj, traj[1]), axis=1)
        self.z_traj = np.concatenate((self.z_traj, traj[2]), axis=1)

        # Update new start point
        self.initial_xyz = end

    def create_ang_traj(self, start, end):
        start_np = np.array(start)
        end_np = np.array(end)

        # 0 Velocity and Acceleration constants
        d = np.zeros(start_np.shape)
        self.interpolate.compute_interpolation_params(start_np, end_np, d, d, d, d, 0, 1)
        time_array = np.arange(0, 1, 0.005)
        traj = self.interpolate.get_interpolated_x(time_array)

        self.ro_traj = np.concatenate((self.ro_traj, traj[0]), axis=1)
        self.pi_traj = np.concatenate((self.pi_traj, traj[1]), axis=1)
        self.ya_traj = np.concatenate((self.ya_traj, traj[2]), axis=1)

        # Update new start point
        self.initial_rpy = end

    def create_grip_traj(self, start, end):
        start_np = np.array(start)
        end_np = np.array(end)

        # 0 Velocity and Acceleration constants
        d = np.zeros(start_np.shape)
        self.interpolate.compute_interpolation_params(start_np, end_np, d, d, d, d, 0, 1)
        time_array = np.arange(0, 1, 0.005)
        traj = self.interpolate.get_interpolated_x(time_array)

        self.gr_traj = np.concatenate((self.gr_traj, traj[0]), axis=1)

        # Update new start point
        self.initial_gr = end

    def update(self):
        if self.count < self.time:
            # Update Position and Angle of PSM
            self.x = self.x_traj[0, self.count]
            self.y = self.y_traj[0, self.count]
            self.z = self.z_traj[0, self.count]
            
            self.ro = self.ro_traj[0, self.count]
            self.pi = self.pi_traj[0, self.count]
            self.ya = self.ya_traj[0, self.count]

            self.gr = self.gr_traj[0, self.count]
            self.count += 1

class PSMControllerDP:
    def __init__(self, dp_handle, arm):
        self.counter = 0
        self.dp = dp_handle
        self.arm = arm

    def update_arm_pose(self):
        dp = self.dp
        dp.update()
        T_t_b = Frame(Rotation.RPY(dp.ro, dp.pi, dp.ya), Vector(dp.x, dp.y, dp.z))
        self.arm.servo_cp(T_t_b)
        self.arm.set_jaw_angle(dp.gr)
        self.arm.run_grasp_logic(dp.gr)

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.arm.target_IK is not None:
            dp = self.dp
            T_ik_w = self.arm.get_T_b_w() * Frame(Rotation.RPY(dp.ro, dp.pi, dp.ya), Vector(dp.x, dp.y, dp.z))
            self.arm.target_IK.set_pose(T_ik_w)
        if self.arm.target_FK is not None:
            ik_solution = self.arm.get_ik_solution()
            ik_solution = np.append(ik_solution, 0)
            T_t_b = convert_mat_to_frame(self.arm.compute_FK(ik_solution))
            T_t_w = self.arm.get_T_b_w() * T_t_b
            self.arm.target_FK.set_pose(T_t_w)

    def run(self):
            self.update_arm_pose()
            self.update_visual_markers()


class PSMController:
    def __init__(self, gui_handle, arm):
        self.counter = 0
        self.GUI = gui_handle
        self.arm = arm

    def update_arm_pose(self):
        gui = self.GUI
        gui.App.update()
        T_t_b = Frame(Rotation.RPY(gui.ro, gui.pi, gui.ya), Vector(gui.x, gui.y, gui.z))
        self.arm.servo_cp(T_t_b)
        self.arm.set_jaw_angle(gui.gr)
        self.arm.run_grasp_logic(gui.gr)

    def update_visual_markers(self):
        # Move the Target Position Based on the GUI
        if self.arm.target_IK is not None:
            gui = self.GUI
            T_ik_w = self.arm.get_T_b_w() * Frame(Rotation.RPY(gui.ro, gui.pi, gui.ya), Vector(gui.x, gui.y, gui.z))
            self.arm.target_IK.set_pose(T_ik_w)
        if self.arm.target_FK is not None:
            ik_solution = self.arm.get_ik_solution()
            ik_solution = np.append(ik_solution, 0)
            T_t_b = convert_mat_to_frame(self.arm.compute_FK(ik_solution))
            T_t_w = self.arm.get_T_b_w() * T_t_b
            self.arm.target_FK.set_pose(T_t_w)

    def run(self):
            self.update_arm_pose()
            self.update_visual_markers()


class ECMController:
    def __init__(self, gui, ecm):
        self.counter = 0
        self._ecm = ecm
        self._cam_gui = gui

    def update_camera_pose(self):
        self._cam_gui.App.update()
        self._ecm.servo_jp(self._cam_gui.jnt_cmds)

    def run(self):
            self.update_camera_pose()


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='ambf_client')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Control PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Control PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Control PSM3', default=False)
    parser.add_argument('--ecm', action='store', dest='run_ecm', help='Control ECM', default=True)
    parser.add_argument('--tool_id', action='store', dest='psm_tool_id', help='PSM Tool ID', default=ToolType.Default)

    parsed_args = parser.parse_args()
    print('Specified Arguments')
    print(parsed_args)

    parsed_args.run_psm_one = get_boolean_from_opt(parsed_args.run_psm_one)
    parsed_args.run_psm_two = get_boolean_from_opt(parsed_args.run_psm_two)
    parsed_args.run_psm_three = get_boolean_from_opt(parsed_args.run_psm_three)
    parsed_args.run_ecm = get_boolean_from_opt(parsed_args.run_ecm)
    tool_id = int(parsed_args.psm_tool_id)
    simulation_manager = SimulationManager(parsed_args.client_name)

    time.sleep(0.5)
    controllers = []

    if parsed_args.run_psm_one is True:
        arm_name = 'psm1'
        psm = PSM(simulation_manager, arm_name, tool_id=tool_id)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM1
            # init_xyz = [0.1, -0.85, -0.15]
            init_xyz = [0, 0, -0.10]
            init_rpy = [3.14, 0, 1.57079]
            gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 0.3, 10.0, 0.000001)
            controller = PSMController(gui, psm)
            controllers.append(controller)

    if parsed_args.run_psm_two is True:
        arm_name = 'psm2'
        psm = PSM(simulation_manager, arm_name, tool_id=tool_id)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -0.10]
            init_rpy = [3.14, 0, 1.57079]
            init_gr = [0.0]
            #gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 0.3, 12, 0.000001)
            #controller = PSMController(gui, psm)
            dp = DP0(arm_name + '/baselink', init_xyz, init_rpy, init_gr, 0.3, 10.0, 0.000001)
            controller = PSMControllerDP(dp, psm)
            controllers.append(controller)

    if parsed_args.run_psm_three is True:
        arm_name = 'psm3'
        psm = PSM(simulation_manager, arm_name, tool_id=tool_id)
        if psm.base is not None:
            print('LOADING CONTROLLER FOR ', arm_name)
            # Initial Target Offset for PSM2
            init_xyz = [0, 0.0, -0.10]
            init_rpy = [3.14, 0, 1.57079]
            gui = ObjectGUI(arm_name + '/baselink', init_xyz, init_rpy, 0.3, 3.14, 0.000001)
            controller = PSMController(gui, psm)
            controllers.append(controller)

    if parsed_args.run_ecm is True:
        arm_name = 'CameraFrame'
        ecm = ECM(simulation_manager, arm_name)
        gui = JointGUI('ECM JP', 4, ["ecm j0", "ecm j1", "ecm j2", "ecm j3"], lower_lims=ecm.get_lower_limits(),
                       upper_lims=ecm.get_upper_limits())
        controller = ECMController(gui, ecm)
        controllers.append(controller)

    if len(controllers) == 0:
        print('No Valid PSM Arms Specified')
        print('Exiting')

    else:
        while not rospy.is_shutdown():
            for cont in controllers:
                cont.run()
            time.sleep(0.005)