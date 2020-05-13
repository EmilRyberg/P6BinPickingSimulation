import sys
#sys.path.append('controllers/ur_controller')
#sys.path.append('controllers/ur_controller/P6BinPicking')

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
from controllers.ur_controller.P6BinPicking.vision.surface_normal import SurfaceNormals
from scipy.spatial.transform import Rotation

from controllers.ur_controller.kinematics.forward import ForwardKinematics
from controllers.ur_controller.kinematics.inverse import InverseKinematics
from controllers.ur_controller.ur_utils import Utils


class SimulationConnector:
    def __init__(self, port):
        self.port = port
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn.connect(('127.0.0.1', port))
        print("Connected on port " + str(port))
        self.box_detector = BoxDetector()
        self.suction_enable_pin = 6
        self.home_pose_l = [35, -300, 300, 0, 0, -0.8]
        #self.home_pose = [-60, -60, -110, -100, 90, -60]
        self.home_pose = [-60, -60, -110, -190, -70, 100]
        #self.move_out_of_view_pose = [-150, -60, -110, -100, 90, -60]
        self.move_out_of_view_pose = [-150, -60, -110, -190, -70, 100]
        self.default_orientation = [0, 0, 0]
        self.gripper_tcp = [0, 0, 0.201, 0, 0, 0]
        self.suction_tcp = [-0.193, 0, 0.08, 0, -np.pi/2, 0]

        self.camera_pose_gripper = [-60, -60, -110, -100, -90, -75]
        self.camera_pose_suction = [-5, -40, -100, -140, 0, -170]

        self.pcb_singularity_avoidance = [-70, -70, -107, -180, -147, 90]

        self.cover_closed = 20
        #self.cover_finger_0 = 0.004 #positions for gripper motors
        #self.cover_finger_1 = 0.006
        #self.cover_grasped = -0.005
        self.box_closed = 3
        #self.box_finger_0 = 0.005
        #self.box_finger_1 = 0.012
        #self.box_grasped = -0.005


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

    def movel2(self, location, orientation, acc=1.0, vel=0.2, wait=None):
        self.movel(np.concatenate((location, orientation)), acc=acc, vel=vel, wait=wait)

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

    def open_gripper(self):
        cmd = {"name": "open_gripper", "args": {}}
        self._execute_remote_command(cmd)

    def close_gripper(self, width=0, speed=5):
        cmd = {"name": "close_gripper", "args": {
            "width": width,
            "speed": speed
        }}
        self._execute_remote_command(cmd)

    def grasp_cover(self):
        self.close_gripper(self.cover_closed)

    def grasp_box(self):
        self.close_gripper(self.box_closed)

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

    def get_gripper_distance(self):
        cmd = {"name": "finger_displacement", "args": {}}
        distance = self._execute_remote_command(cmd)
        return distance


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
    connector = SimulationConnector(2000)
    connector.move_to_home()
    connector.set_tcp(connector.suction_tcp)
    connector.movej([-60, -60, -110, -190, -70, 100], vel=3)

    #connector.movel([0, -300, 300, 0, 3.14, 0], vel=0.8)


    time.sleep(10)
