"""ur_controller controller."""

from controller import Robot
from controller import Connector
from controller import RangeFinder
from controller import Camera
import numpy as np
from kinematics.inverse import InverseKinematics
from kinematics.forward import ForwardKinematics
from trajectory import Trajectory
from utils import Utils
from scipy.spatial.transform import Rotation
import socket
import time
import struct
import pickle
from PIL import Image as pimg
np.set_printoptions(precision=4, suppress=True)

def send_msg(msg):
    # Prefix each message with a 4-byte length (network byte order)
    msg = struct.pack('>I', len(msg)) + msg
    conn.setblocking(True)
    conn.sendall(msg)
    conn.setblocking(False)

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
        packet = None
        try:
            packet = conn.recv(n - len(data))
        except socket.error:
            pass
        if not packet:
            return None
        data.extend(packet)
    return data

def get_joint_angles():
    angles = np.zeros(6)
    for i in range(6):
        angles[i] = motor_sensors[i].getValue()
    return angles

def respond(result, data = None):
    cmd = {}
    cmd["result"] = result
    cmd["data"] = data
    send_msg(pickle.dumps(cmd))


robot = Robot()
suction = Connector("suction")
cameraRGB = robot.getCamera("cameraRGB")
cameraDepth = robot.getRangeFinder("cameraDepth")


timestep = int(robot.getBasicTimeStep())

motors = [robot.getMotor("shoulder_pan_joint"), robot.getMotor("shoulder_lift_joint"), robot.getMotor("elbow_joint"),
          robot.getMotor("wrist_1_joint"), robot.getMotor("wrist_2_joint"), robot.getMotor("wrist_3_joint"), robot.getMotor("rotational motor")]
motor_sensors = [robot.getPositionSensor("shoulder_pan_joint_sensor"), robot.getPositionSensor("shoulder_lift_joint_sensor"), robot.getPositionSensor("elbow_joint_sensor"),
          robot.getPositionSensor("wrist_1_joint_sensor"), robot.getPositionSensor("wrist_2_joint_sensor"), robot.getPositionSensor("wrist_3_joint_sensor")]

finger_motors = [robot.getMotor("right_finger_motor"), robot.getMotor("left_finger_motor")]
finger_sensors = [robot.getPositionSensor("right_finger_sensor"), robot.getPositionSensor("left_finger_sensor")]

for sensor in motor_sensors:
    sensor.enable(10)
motors[0].setPosition(1.57)
motors[1].setPosition(-2.14)
motors[2].setPosition(-1.57)
motors[3].setPosition(-1.01)
motors[4].setPosition(1.57)
motors[5].setPosition(1.05)
motors[6].setPosition(0.35)
first_run = True
can_run = False
time_passed = 0
inv_index = 0
inverse_solution = None
wait_time = 2000
inv_test = False
trajectory_test = False
ikin = InverseKinematics()
fkin = ForwardKinematics()
trajectory = Trajectory(motor_sensors, fkin, timestep)
current_task = "idle"
args = None
command_is_executing = False
print_once_flag = True
rgb_enabled = False
depth_enabled = False

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('localhost', 2000))
server.listen()
print("Waiting for connection")
robot.step(1) # webots won't print without a step
conn, addr = server.accept()
#conn.settimeout(0.1)
conn.setblocking(False)
print("Connected")

#Init loop
# while robot.step(timestep) != -1:
#     time_passed += timestep
#     if time_passed > wait_time:
#         break

#trajectory.generate_trajectory([-0.13, 0.16, 0.7, 0.5, -1.5, 0.5], 0.1)

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
            #print(cmd)
            current_task = cmd["name"]
            args = cmd["args"]
            print("Executing: " + current_task + " " + str(args))
            print_once_flag = True

    elif current_task == "movej":
        if not command_is_executing:
            for i in range(6):
                motors[i].setVelocity(args["speed"])
                motors[i].setPosition(args["angles"][i])
            command_is_executing = True
        else:
            diff = 0
            for i in range(6):
                diff += abs(args["angles"][i] - motor_sensors[i].getValue())
            if diff < 0.1:
                command_is_executing = False
                current_task = "idle"
                respond("done")

    elif current_task == "suction_on":
        suction.lock()
        current_task = "idle"
        respond("done")

    elif current_task == "suction_off":
        suction.unlock()
        current_task = "idle"
        respond("done")

    if current_task == "close_gripper":
        finger_motors[1].setVelocity(5)
        finger_motors[1].setPosition(0.006)
        finger_motors[0].setVelocity(5)
        finger_motors[0].setPosition(0.004)
        finger_sensors[0].enable(1)
        for i in range(1000):
            print(finger_sensors[0].getValue())
            command_is_executing = True
            if finger_sensors[0].getValue()+0.0005>=-0.005:
                command_is_executing = False
                continue
            robot.step(1)
        finger_sensors[0].disable()
        current_task = "idle"
        respond("done")

    if current_task == "open_gripper":
        finger_motors[1].setVelocity(5)
        finger_motors[1].setPosition(0.045)
        finger_motors[0].setVelocity(5)
        finger_motors[0].setPosition(-0.035)
        finger_sensors[0].enable(1)
        for i in range(1000):
            print(finger_sensors[0].getValue())
            command_is_executing = True
            if -0.35<=finger_sensors[0].getValue():
                continue
            robot.step(1)
        finger_sensors[0].disable()
        current_task = "idle"
        respond("done")

    if current_task == "movel":
        if not command_is_executing:
            trajectory.generate_trajectory(args["coords"], args["speed"])
            command_is_executing = True
        else:
            angles = trajectory.calculate_step()
            for i in range(6):
                motors[i].setPosition(angles[i])
            if trajectory.is_done:
                command_is_executing = False
                current_task = "idle"
                respond("done")

    elif current_task == "getl":
        thetas = [0]*6
        for i in range(6):
            thetas[i] = motor_sensors[i].getValue()
        tmat = fkin.compute_TBT(thetas)
        trans, rot = Utils.tmat_to_trans_and_rot(tmat)
        rotvec = rot.as_rotvec()
        pose = [trans[0], trans[1], trans[2], rotvec[0], rotvec[1], rotvec[2]]
        respond("done", pose)
        current_task = "idle"

    elif current_task == "set_tcp":
        pose = args["pose"]
        trans = [pose[0], pose[1], pose[2]]
        rotvec = [pose[3], pose[4], pose[5]]
        rot = Rotation.from_rotvec(rotvec)
        tmat = Utils.trans_and_rot_to_tmat(trans, rot)
        fkin.T6T = tmat
        current_task = "idle"
        respond("done")

    elif current_task == "get_image":
        if not rgb_enabled:
            cameraRGB.enable(timestep)
            rgb_enabled = True
        else:
            np_img = np.array(cameraRGB.getImageArray(), dtype=np.uint8)
            respond("done", np_img)
            current_task = "idle"
            cameraRGB.disable()
            rgb_enabled = False

    elif current_task == "get_depth":
        if not depth_enabled:
            cameraDepth.enable(timestep)
            depth_enabled = True
        else:
            np_dep = np.array(cameraDepth.getRangeImageArray())
            respond("done", np_dep)
            current_task = "idle"
            cameraDepth.disable()
            depth_enabled = False

    else:
        respond("Unknown command: " + current_task)
        raise Exception("Received unknown command: " + current_task)


conn.close()
print("Robot controller ended")