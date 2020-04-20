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


timestep = int(robot.getBasicTimeStep())

motors = [robot.getMotor("shoulder_pan_joint"), robot.getMotor("shoulder_lift_joint"), robot.getMotor("elbow_joint"),
          robot.getMotor("wrist_1_joint"), robot.getMotor("wrist_2_joint"), robot.getMotor("wrist_3_joint")]
motor_sensors = [robot.getPositionSensor("shoulder_pan_joint_sensor"), robot.getPositionSensor("shoulder_lift_joint_sensor"), robot.getPositionSensor("elbow_joint_sensor"),
          robot.getPositionSensor("wrist_1_joint_sensor"), robot.getPositionSensor("wrist_2_joint_sensor"), robot.getPositionSensor("wrist_3_joint_sensor")]


trajectory = Trajectory(motor_sensors, timestep)

for sensor in motor_sensors:
    sensor.enable(10)
motors[0].setPosition(np.pi / 2)
motors[1].setPosition(-np.pi*0.7)
motors[2].setPosition(-np.pi / 2)
motors[3].setPosition(-np.pi*0.3)
motors[4].setPosition(np.pi / 2)
motors[5].setPosition(np.pi / 2)
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
current_task = "idle"
args = None
command_is_executing = False
print_once_flag = True

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('localhost', 2000))
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

trajectory.generate_trajectory([-0.13, 0.16, 0.7, 0.5, -1.5, 0.5], 0.1)

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

    if current_task == "movej":
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

    if current_task == "suction_on":
        suction.lock()
        current_task = "idle"
        respond("done")

    if current_task == "suction_off":
        suction.unlock()
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

    if current_task == "get_image":
        pil_img = pimg.open("test.png")
        np_img = np.asarray(pil_img)
        respond("done", np_img)
        current_task = "idle"

    if trajectory_test:
        angles = trajectory.calculate_step()
        #print("step ", trajectory.current_step, "out of ", trajectory.total_steps)
        for i in range(6):
            motors[i].setPosition(angles[i])
        if trajectory.is_done:
            trajectory_test = False

conn.close()
print("Controller ended")