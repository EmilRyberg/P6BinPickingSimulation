import sys
sys.path.append('controllers/ur_controller')
sys.path.append('controllers/ur_controller/P6BinPicking')

import numpy as np
import threading
import socket
import cv2
import time
import struct
import pickle
from PIL import Image as pimg
import math
from controllers.ur_controller.P6BinPicking.vision.box_detector import BoxDetector
from controllers.ur_controller.P6BinPicking.aruco import Calibration
from controllers.ur_controller.P6BinPicking.vision.surface_normal import SurfaceNormals
from scipy.spatial.transform import Rotation

from controllers.ur_controller.kinematics.forward import ForwardKinematics
from controllers.ur_controller.kinematics.inverse import InverseKinematics
from controllers.ur_controller.utils import Utils


class SimulationConnector:
    def __init__(self, port):
        self.port = port
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn.connect(('127.0.0.1', port))
        print("Connected on port " + str(port))
        self.box_detector = BoxDetector()
        self.calibration = Calibration()

        self.suction_enable_pin = 6
        self.home_pose_l = [35, -300, 300, 0, 0, -0.8]
        self.home_pose = [-60, -60, -110, -100, 90, -60]
        # self.move_out_of_view_pose = [-350, -35, 300, 3.14, 0, 0]
        self.move_out_of_view_pose = [-150, -60, -110, -100, 90, -60]
        self.default_orientation = [0, 0, 0]
        self.gripper_tcp = [0, 0, 0.201, 2.9024, -1.2023, 0]
        self.fuse_tcp = [0.057, -0.00109, 0.13215, -1.7600, -0.7291, 1.7601]
        # self.suction_tcp_real = [-0.12, 0, 0.095, 0, -1.57, 0]
        #self.suction_tcp = [0, 0.18, 0.095, 1.57, 0, 0]
        self.suction_tcp = [0, 0.193, 0.08, -np.pi/2, 0, 0]
        self.current_part_id = None
        self.grip_has_been_called_flag = False
        self.moved_to_camera_flag = False

        self.align_fuse_point_1 = [258.3808266269915, 182.66080196127277, 50.755338740619685, 0.5129225399673327,
                                   -0.5681073061405235, -0.021312928850932115]
        self.align_fuse_point_2 = [269.56669707049855, 192.5271576116136, 36.52450508933613, 0.5352153513500437,
                                   -0.5851532800726972, -0.022296119825804664]
        self.align_fuse_point_3 = [256.79863096450663, 180.323917259804, 38.04563142826764, 0.5128000908988749,
                                   -0.5681263381546497, -0.021276095366553276]
        self.align_fuse_point_4 = [267.24826717836964, 189.57576475222854, 23.53934054782497, 0.5129307828342723,
                                   -0.5681049276113338, -0.02124040568020565]

        self.align_pcb_1 = [2, -62, -108, -97, 89, 46]  # joint values
        self.align_pcb_2 = [381, -12, 272, 0.61, -1.51, 0.64]  # Cartesian coordinates
        self.align_pcb_3 = [388, 2, 280, 0.6154, -1.5228, 0.62]  # Cartesian coordinates
        self.align_pcb_4 = [-6, -51, -114, -96, 89, 44]

        self.align_pcb_flipped_1 = [22, -86, -84, -98, 89, -110]
        self.align_pcb_flipped_2 = [373, -16.5, 257, 2.405, 1.018, 2.52]  # Cartesian coordinates
        self.align_pcb_flipped_3 = [373, -16.5, 300, 2.405, 1.018, 2.52]  # Cartesian coordinates
        self.align_pcb_flipped_4 = [-16, -55, -107, -104, 87, 29]

        self.align_pcb_pick_1 = [22, -57, -150, -70, 53, 61]  # Joint values
        self.align_pcb_pick_2 = [362, -23, 47, 0.177, -1.1775, 1.194]  # Cartesian coordinates
        self.align_pcb_pick_3 = [336.5, -47, 67.5, 0.177, -1.1775, 1.194]  # Cartesian

        self.align_cover_1 = [122.5, -71.6, 105.5, -84, -13, -60]  # joint
        self.align_cover_2 = [560, -253, 134.7, 1.4433, -0.333, -1.095]  # Cartesian

        self.align_cover_flipped_1 = [3.5, -143, -45, -186, 53, 120]  # joint
        self.align_cover_flipped_2 = [546, -265, 136.5, 3.124, 0.44, 2.146]  # Cartesian
        self.align_cover_flipped_3 = [113, -75, 105, -83, -19, -61]

        self.align_cover_pick_1 = [531, -265, 160, 1.32, -0.334, -1.13]  # Cartesian
        self.align_cover_pick_2 = [539.9, -287.6, 99.3, 1.397, -0.316, -1.096]  # Cartesian
        self.align_cover_pick_3 = [563.4, -268.1, 92.3, 1.397, -0.316, -1.096]  # Cartesian
        self.align_cover_pick_4 = [564.5, -267.4, 93.6, 1.402, -0.343, -1.087]
        self.align_cover_pick_5 = [539, -287, 101, 1.402, -0.343, -1.087]
        self.align_cover_pick_6 = [-130, -103, 155, -180, 1, -60]
        self.align_cover_pick_7 = [-130, -100, -130, -180, 1, -60]
        self.align_cover_pick_8 = [-61, -62, -107, -100, 89, -61]  # joint

        self.test_back_loc = (-255, -280)
        self.test_pcb_loc = (-174, -362)
        self.test_fuse_1_loc = (-138, -278)
        self.test_fuse_2_loc = (-100, -315)
        self.test_top_loc = (-107, -425)

        self.camera_pose_gripper = [-60, -60, -110, -100, -90, -75]
        self.camera_pose_suction = [-5, -40, -100, -140, 0, -170]

        self.pcb_singularity_avoidance = [-70, -70, -107, -180, -147, 90]

        self.cover_finger_0 = 0.004 #positions for gripper motors
        self.cover_finger_1 = 0.006
        self.cover_grasped = -0.005
        self.box_finger_0 = 0.005
        self.box_finger_1 = 0.012
        self.box_grasped = -0.005
        self.first_box_move = 0

    def __del__(self):
        self.conn.close()
        print("Closed connection on port " + str(self.port))

    def _send_msg(self, msg):
        # Prefix each message with a 4-byte length (network byte order)
        msg = struct.pack('>I', len(msg)) + msg
        self.conn.sendall(msg)

    def _recv_msg(self):
        # Read message length and unpack it into an integer
        raw_msglen = self._recvall(4)
        if not raw_msglen:
            return None
        msglen = struct.unpack('>I', raw_msglen)[0]
        # Read the message data
        return self._recvall(msglen)

    def _recvall(self, n):
        # Helper function to recv n bytes or return None if EOF is hit
        data = bytearray()
        while len(data) < n:
            packet = self.conn.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data

    def _execute_remote_command(self, command):
        self._send_msg(pickle.dumps(command))
        response = self._recv_msg()
        response = pickle.loads(response)
        if response["result"] != "done":
            raise Exception("Unexpected response: " + response["result"])
        else:
            print("cmd done")
        return response["data"]

    def movej(self, pose, acc=1.0, vel=0.1, degrees=True, wait=None):
        pose_local = pose.copy()
        # print("pose in deg: ", pose_local)
        if degrees:
            for i in range(6):
                pose_local[i] = math.radians(pose_local[i])
        # print("pose in radians: ", pose_local)
        cmd = {"name": "movej", "args": {}}
        cmd["args"]["angles"] = pose_local
        cmd["args"]["speed"] = vel
        cmd["args"]["acc"] = -1
        self._execute_remote_command(cmd)

    def movel(self, pose, acc=1.0, vel=0.2, wait=None):
        pose_local = pose.copy()
        print("goal pose in mm: ", pose_local)
        pose_local[0] *= 0.001
        pose_local[1] *= 0.001
        pose_local[2] *= 0.001
        # print(pose_local)
        cmd = {"name": "movel", "args": {}}
        cmd["args"]["coords"] = pose_local
        cmd["args"]["speed"] = vel
        self._execute_remote_command(cmd)

    def set_tcp(self, pose):
        cmd = {"name": "set_tcp", "args": {"pose": pose}}
        self._execute_remote_command(cmd)

    def getl(self):
        cmd = {"name": "getl", "args": {}}
        return self._execute_remote_command(cmd)

    def move_to_home(self, speed=1.0):
        self.movej(self.home_pose, acc=1.0, vel=speed)

    def move_to_home_l(self, speed=1.0):
        self.movel(self.home_pose_l, acc=1.0, vel=speed)

    def move_to_camera(self, speed=1.0, is_pcb=False):
        self.moved_to_camera_flag = True
        if is_pcb:
            self.movej(self.camera_pose_suction, acc=1.0, vel=speed)
        else:
            self.movej(self.camera_pose_gripper, acc=1.0, vel=speed)

    def move_out_of_view(self, speed=1.0):
        self.movej(self.move_out_of_view_pose, acc=1.0, vel=speed)

    def open_gripper(self, width=100):
        cmd = {"name": "open_gripper", "args": {}}
        self._execute_remote_command(cmd)

    def close_gripper(self, finger_0_position, finger_1_position, closed_threshold):
        cmd = {"name": "close_gripper", "args": {}}
        cmd["args"]["finger_0"] = finger_0_position
        cmd["args"]["finger_1"] = finger_1_position
        cmd["args"]["closed"] = closed_threshold
        self._execute_remote_command(cmd)

    def grasp_cover(self):
        self.close_gripper(self.cover_finger_0,self.cover_finger_1,self.cover_grasped)

    def grasp_box(self):
        self.close_gripper(self.box_finger_0, self.box_finger_1, self.box_grasped)

    def move_gripper(self, width):
        # WIP
        pass

    def enable_suction(self):
        cmd = {"name": "suction_on", "args": {}}
        self._execute_remote_command(cmd)

    def disable_suction(self):
        cmd = {"name": "suction_off", "args": {}}
        self._execute_remote_command(cmd)

    def get_image(self):
        cmd = {"name": "get_image", "args": {}}
        np_img = self._execute_remote_command(cmd)
        np_img = np_img.transpose((1, 0, 2))
        np_img = np_img[:, :, ::-1]
        return np_img

    def get_depth(self):
        cmd = {"name": "get_depth", "args": {}}
        np_img = self._execute_remote_command(cmd)
        np_img = np_img.transpose((1, 0))
        np_img = np_img * 100
        return np_img

    def get_instance_segmentation(self):
        cmd = {"name": "inst_seg", "args": {}}
        results = self._execute_remote_command(cmd)
        return results
    def move_box(self):
        pil,image = self.get_image()
        #cv2.imshow("box_image", image)
        #cv2.waitKey()
        box_location = self.box_detector.find_box(image)
        grasp_x = box_location[2][0]+int(((box_location[3][0]-box_location[2][0])/2))
        grasp_y = box_location[2][1]+int(((box_location[3][1]-box_location[2][1])/2))
        vector = [box_location[3][0]-box_location[2][0], box_location[3][1]-box_location[2][1]]
        box_start_vector = [0, -483] #hardcoded
        dot_product = np.array(box_start_vector) @ np.array(vector)
        norm_dot_product = np.linalg.norm(np.array(box_start_vector) * np.linalg.norm(np.array(vector)))
        angle = np.arccos(dot_product / norm_dot_product)
        #print(angle)
        #print(grasp_x,grasp_y)
        grasp_location = self.calibration.calibrate(pil,grasp_x,grasp_y, 130)
        self.set_tcp(self.gripper_tcp)
        self.move_to_home()
        self.movel([grasp_location[0], grasp_location[1], grasp_location[2]+20, 0, 0, 3.14-angle])
        self.movel([grasp_location[0], grasp_location[1], grasp_location[2]-80, 0, 0, 3.14-angle])
        self.grasp_box()
        if self.first_box_move == 0:
            self.movel([grasp_location[0] + 15, grasp_location[1] + 15, grasp_location[2] + 20, 0, 0, 3.14])
            self.first_box_move = 1
        else:
            self.movel([grasp_location[0]-15, grasp_location[1]-15, grasp_location[2]+20, 0, 0, 3.14])
            self.first_box_move = 0
        self.open_gripper()
        #print(grasp_location)


def angle_axis_to_rotation_matrix(angle_axis):
    c = np.cos(angle_axis[3])
    s = np.sin(angle_axis[3])
    t = 1 - c
    x = angle_axis[0]
    y = angle_axis[1]
    z = angle_axis[2]
    print('aa', angle_axis)
    rot = [[t * x ** 2 + c, t * x * y - z * s, t * x * z + y * s],
           [t * x * y + z * s, t * y ** 2 + c, t * y * z - x * s],
           [t * x * z - y * s, t * y * z + x * s, t * z ** 2 + c]]
    print('a', rot)
    return np.array(rot)


if __name__ == '__main__':
    ikin = InverseKinematics()
    fkin = ForwardKinematics()
    connector = SimulationConnector(2000)
    connector.move_out_of_view()
    np_rgb_img = connector.get_image()
    np_depth_img = connector.get_depth()
    results = connector.get_instance_segmentation()
    first_mask = results["instances"].pred_masks[0, ::].numpy().astype(np.uint8)
    surface_normals = SurfaceNormals()
    center, rotation_matrix = surface_normals.get_tool_orientation_matrix(first_mask, np_depth_img, np_rgb_img)
    TBT = np.pad(rotation_matrix, ((0, 1), (0, 1)))
    TBT[0, 3] = center[0] / 1000.0
    TBT[1, 3] = center[1] / 1000.0
    TBT[2, 3] = 0.3 # center[2] / 1000.0 + 0.2
    TBT[3, 3] = 1
    #connector.set_tcp(connector.suction_tcp)
    pose = connector.suction_tcp
    trans = [pose[0], pose[1], pose[2]]
    rotvec = [pose[3], pose[4], pose[5]]
    rot = Rotation.from_rotvec(rotvec)
    tmat = Utils.trans_and_rot_to_tmat(trans, rot)
    fkin.T6T = tmat
    T06 = fkin.convert_TBT_to_T06(TBT)
    angles = ikin.get_best_solution_for_config_id(T06, 0)

    connector.movej(angles, acc=2, vel=0.5, degrees=False)

    time.sleep(10)
