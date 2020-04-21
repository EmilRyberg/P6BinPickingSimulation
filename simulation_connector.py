import numpy as np
import threading
import socket
import time
import struct
import pickle
from PIL import Image as pimg

class Command:
    def __init__(self):
        self.name = None
        self.args = {}
        self.port = None

class SimulationConnector:
    def __init__(self, port):
        self.port = port
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn.connect(('127.0.0.1', port))
        print("Connected on port " + str(port))

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
            raise Exception("Unexpected response: " + response)
        else:
            print("cmd done")
        return response["data"]

    def movej(self, angles, speed):
        cmd = {"name" : "movej", "args" : {}}
        cmd["args"]["angles"] = angles
        cmd["args"]["speed"] = speed
        self._execute_remote_command(cmd)

    def movel(self, coords, speed):
        cmd = {"name" : "movel", "args" : {}}
        cmd["args"]["coords"] = coords
        cmd["args"]["speed"] = speed
        self._execute_remote_command(cmd)

    def get_image(self):
        cmd = {"name": "get_image", "args": {}}
        np_img = self._execute_remote_command(cmd)
        pil_img = pimg.fromarray(np_img)
        pil_img.show()

    def get_depth(self):
        cmd = {"name": "get_depth", "args": {}}
        np_img = self._execute_remote_command(cmd)
        np_img = np_img * 85
        pil_img = pimg.fromarray(np_img)
        pil_img.show()

    def suction_handler(self, option):
        if option == 1:
            cmd = {"name" : "suction_on", "args" : {}}
            self._execute_remote_command(cmd)
        elif option == 0:
            cmd = {"name" : "suction_off", "args" : {}}
            self._execute_remote_command(cmd)
        else:
            raise Exception("wrong option for suction: " + option)

if __name__ == '__main__':
    connector = SimulationConnector(2000)
    camera = SimulationConnector(2001)
    camera.get_depth()
    camera.get_image()
    #connector.get_image()
    #connector.movel([-0.25, 0.30, 1.11, 0.5, -1.5, 0.5], 0.3)
    #connector.movej([0,0,0,0,0,0], 1)
    #connector.suction_handler(1)
    #connector.movel([-0.25, 0.30, 1.11, 0.5, -1.5, 0.5], 0.3)
    time.sleep(10)