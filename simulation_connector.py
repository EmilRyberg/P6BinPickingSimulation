import numpy as np
import threading
import socket
import time
import struct
import pickle
from PIL import Image as pimg
import math
from controllers.ur_controller.P6BinPicking.vision.surface_normal import SurfaceNormals
from scipy.spatial.transform import Rotation

class SimulationConnector:
    def __init__(self, port):
        self.port = port
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn.connect(('127.0.0.1', port))
        print("Connected on port " + str(port))


        self.suction_enable_pin = 6
        self.home_pose_l = [35, -300, 300, 0, 0, -0.8]
        self.home_pose = [-60, -60, -110, -100, 90, -60]
        # self.move_out_of_view_pose = [-350, -35, 300, 3.14, 0, 0]
        self.move_out_of_view_pose = [-150, -60, -110, -100, 90, -60]
        self.default_orientation = [0, 0, 0]
        self.gripper_tcp = [0, 0, 0.201, 2.9024, -1.2023, 0]
        self.fuse_tcp = [0.057, -0.00109, 0.13215, -1.7600, -0.7291, 1.7601]
        #self.suction_tcp_real = [-0.12, 0, 0.095, 0, -1.57, 0]
        self.suction_tcp = [0, 0.18, 0.095, 1.57, 0, 0]
        self.current_part_id = None
        self.grip_has_been_called_flag = False
        self.moved_to_camera_flag = False

        self.align_fuse_point_1 = [258.3808266269915, 182.66080196127277, 50.755338740619685, 0.5129225399673327, -0.5681073061405235, -0.021312928850932115]
        self.align_fuse_point_2 = [269.56669707049855, 192.5271576116136, 36.52450508933613, 0.5352153513500437, -0.5851532800726972, -0.022296119825804664]
        self.align_fuse_point_3 = [256.79863096450663, 180.323917259804, 38.04563142826764, 0.5128000908988749, -0.5681263381546497, -0.021276095366553276]
        self.align_fuse_point_4 = [267.24826717836964, 189.57576475222854, 23.53934054782497, 0.5129307828342723, -0.5681049276113338, -0.02124040568020565]

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

    def movej(self, pose, acc=1.0, vel=0.1, wait=None):
        pose_local = pose.copy()
        #print("pose in deg: ", pose_local)
        for i in range(6):
            pose_local[i] = math.radians(pose_local[i])
        #print("pose in radians: ", pose_local)
        cmd = {"name" : "movej", "args" : {}}
        cmd["args"]["angles"] = pose_local
        cmd["args"]["speed"] = vel
        self._execute_remote_command(cmd)

    def movel(self, pose, acc=1.0, vel=0.2, wait=None):
        pose_local = pose.copy()
        print("goal pose in mm: ", pose_local)
        pose_local[0] *= 0.001
        pose_local[1] *= 0.001
        pose_local[2] *= 0.001
        #print(pose_local)
        cmd = {"name" : "movel", "args" : {}}
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

    def close_gripper(self, width=0):
        cmd = {"name": "close_gripper", "args": {}}
        self._execute_remote_command(cmd)

    def move_gripper(self, width):
        #WIP
        pass

    def enable_suction(self):
        cmd = {"name" : "suction_on", "args" : {}}
        self._execute_remote_command(cmd)

    def disable_suction(self):
        cmd = {"name" : "suction_off", "args" : {}}
        self._execute_remote_command(cmd)

    def get_image(self):
        cmd = {"name": "get_image", "args": {}}
        np_img = self._execute_remote_command(cmd)
        np_img = np_img.transpose((1,0,2))
        pil_img = pimg.fromarray(np_img)
        #pil_img = pil_img.transpose(pimg.FLIP_LEFT_RIGHT)
        #pil_img.show()
        return np.asarray(pil_img)

    def get_depth(self):
        cmd = {"name": "get_depth", "args": {}}
        np_img = self._execute_remote_command(cmd)
        np_img = np_img.transpose((1, 0))
        np_img = np_img * 100
        pil_img = pimg.fromarray(np_img)
        #pil_img = pil_img.transpose(pimg.FLIP_TOP_BOTTOM)
        #pil_img.show()
        return np.asarray(pil_img)

    def get_instance_segmentation(self):
        cmd = {"name": "inst_seg", "args": {}}
        results = self._execute_remote_command(cmd)
        return results


if __name__ == '__main__':
    connector = SimulationConnector(2000)
    connector.move_out_of_view()
    np_rgb_img = connector.get_image()
    np_depth_img = connector.get_depth()
    print(np_depth_img.shape)
    results = connector.get_instance_segmentation()
    #print('results', results["instances"])
    #print('masks', results["instances"].pred_masks)
    #print('masks', results["instances"].pred_masks.numpy().shape)
    #print('first mask', results["instances"].pred_masks[0, ::].numpy().astype(np.uint8))
    first_mask = results["instances"].pred_masks[0, ::].numpy().astype(np.uint8)
    surface_normals = SurfaceNormals()
    normal_vector = surface_normals.vector_normal(first_mask, np_depth_img, np_rgb_img)
    tool_direction = normal_vector * -1
    matrix = Rotation.from_rotvec(tool_direction)


    connector.set_tcp(connector.suction_tcp)

    time.sleep(10)