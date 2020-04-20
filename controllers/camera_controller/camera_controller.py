"""camera_controller controller."""

from controller import Robot
from controller import RangeFinder
from controller import Camera
import numpy as np
import socket
import struct
import pickle
from PIL import Image as pimg
np.set_printoptions(precision=4, suppress=True)


def send_msg(msg):
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    conn.sendall(msg)

def recv_msg():
    # Read message length and unpack it into an integer
    raw_msglen = recvall(4)
    if not raw_msglen:
        return None
    msglen = struct.unpack('>I', raw_msglen)[0]
    # Read the message data
    return recvall(msglen)

def recvall(n):
    # Helper function to recv n bytes or return None if EOF is hit
    data = bytearray()
    while len(data) < n:
        packet = conn.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

def respond(result, data = None):
    cmd = {}
    cmd["result"] = result
    cmd["data"] = data
    send_msg(pickle.dumps(cmd))


robot = Robot()

cameraRGB = Camera("cameraRGB")
cameraDepth = RangeFinder("cameraDepth")
timestep = int(robot.getBasicTimeStep())

cameraRGB.enable(timestep)
cameraDepth.enable(timestep)


first_run = True
can_run = False

time_passed = 0
wait_time = 2000

current_task = "idle"
args = None
command_is_executing = False
print_once_flag = True

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('localhost', 2001))
server.listen()
print("Waiting for connection")
robot.step(1) # webots won't print without a step
conn, addr = server.accept()
print("Connected")

#Init loop
while robot.step(timestep) != -1:
    time_passed += timestep
    if time_passed > wait_time:
        break


# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if current_task == "idle":
        if print_once_flag:
            print("Waiting for command")
            robot.step(1)
            print_once_flag = False
        msg = recv_msg()
        if msg != None:
            cmd = pickle.loads(msg)
            print(cmd)
            current_task = cmd["name"]
            args = cmd["args"]
            print("Executing: " + current_task + " " + str(args))
            print_once_flag = True

    if current_task == "get_image":
        np_img = cameraRGB.getImageArray()
        respond("done", np_img)
        current_task = "idle"

    if current_task == "get_depth":
        np_dep = cameraDepth.getRangeImageArray()
        respond("done", np_dep)
        current_task = "idle"


#conn.close()
print("Controller ended")