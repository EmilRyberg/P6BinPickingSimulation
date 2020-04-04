"""ur_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from kinematics.inverse import InverseKinematics
from kinematics.forward import ForwardKinematics
from trajectory import Trajectory
from utils import Utils
np.set_printoptions(precision=4, suppress=True)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)


motors = [robot.getMotor("shoulder_pan_joint"), robot.getMotor("shoulder_lift_joint"), robot.getMotor("elbow_joint"),
          robot.getMotor("wrist_1_joint"), robot.getMotor("wrist_2_joint"), robot.getMotor("wrist_3_joint")]
motor_sensors = [robot.getPositionSensor("shoulder_pan_joint_sensor"), robot.getPositionSensor("shoulder_lift_joint_sensor"), robot.getPositionSensor("elbow_joint_sensor"),
          robot.getPositionSensor("wrist_1_joint_sensor"), robot.getPositionSensor("wrist_2_joint_sensor"), robot.getPositionSensor("wrist_3_joint_sensor")]

trajectory = Trajectory(motors, motor_sensors, timestep)

for sensor in motor_sensors:
    sensor.enable(10)
motors[0].setPosition(np.pi / 2)
motors[1].setPosition(-np.pi*0.7)
motors[2].setPosition(-np.pi / 2)
motors[3].setPosition(-np.pi*0.3)
motors[4].setPosition(np.pi / 2)
motors[5].setPosition(np.pi / 2)
#motors[0].setPosition(3.8338037491315133)
first_run = True
can_run = False
time_passed = 0
inv_index = 0
inverse_solution = None
wait_time = 2000
inv_test = False
trajectory_test = True
inv_kin = InverseKinematics()

def get_joint_angles():
    angles = np.zeros(6)
    for i in range(6):
        angles[i] = motor_sensors[i].getValue()
    return angles

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if first_run and can_run:
        kin = ForwardKinematics()
        T06 = kin.compute_0_to_6_matrix(get_joint_angles())
        Utils.print_tmat(T06, 'T06')
        Utils.print_tmat(kin.convert_T06_to_TB6(T06), "TB6")
        goto_pos = np.array([[0, -1, 0, 0.73],
                             [0, 0, -1, 0.07],
                             [1, 0, 0, 0.44],
                             [0, 0, 0, 1]])
        #goto_pos2 = np.array([[ 0, 0, -1,  0.717],
        #                        [0,  1, 0, 0.0627],
        #                        [ 1,  0, 0, -0.133],
        #                        [ 0, 0, 0, 1]])
        goto_pos3 = np.array([[0, 0, 1, 0.917],
                             [0, 1, 0, 0.0627],
                             [-1, 0, 0, -0.133],
                             [0, 0, 0, 1]])
        T06_goto = kin.convert_TB6_to_T06(goto_pos)
        print("T06 goto:" , T06_goto)
        inverse_solution = inv_kin.compute_joint_angles(T06_goto, True)

        first_run = False
        can_run = False
        time_passed = 0
    elif not can_run:
        time_passed += timestep
        if time_passed > wait_time:
            can_run = True
            trajectory.generate_trajectory([-0.13, 0.16, 0.7, 0.5, -1.5, 0.5], 0.1)
            inv_kin.get_current_configuration_id(get_joint_angles())
            #exit()
    elif can_run:
        if inv_test:
            if inv_index == 0:
                if inverse_solution.solution_shoulder_left.is_valid_solution and inverse_solution.solution_shoulder_left.solution_elbow_up.is_valid_solution:
                    print("Showing shoulder left, elbow up:")
                    for i in range(6):
                        motors[i].setPosition(inverse_solution.solution_shoulder_left.solution_elbow_up.thetas[i])
                inv_index += 1
                can_run = False
                time_passed = 0
                wait_time = 4000
            elif inv_index == 1:
                if inverse_solution.solution_shoulder_left.is_valid_solution and inverse_solution.solution_shoulder_left.solution_elbow_down.is_valid_solution:
                    print("Showing shoulder left, elbow down:")
                    for i in range(6):
                        motors[i].setPosition(inverse_solution.solution_shoulder_left.solution_elbow_down.thetas[i])
                inv_index += 1
                can_run = False
                time_passed = 0
            elif inv_index == 2:
                if inverse_solution.solution_shoulder_right.is_valid_solution and inverse_solution.solution_shoulder_right.solution_elbow_up.is_valid_solution:
                    print("Showing shoulder right, elbow up:")
                    for i in range(6):
                        motors[i].setPosition(inverse_solution.solution_shoulder_right.solution_elbow_up.thetas[i])
                inv_index += 1
                can_run = False
                time_passed = 0
            elif inv_index == 3:
                if inverse_solution.solution_shoulder_right.is_valid_solution and inverse_solution.solution_shoulder_right.solution_elbow_down.is_valid_solution:
                    print("Showing shoulder right, elbow down:")
                    for i in range(6):
                        motors[i].setPosition(inverse_solution.solution_shoulder_right.solution_elbow_down.thetas[i])
                inv_index += 1
                can_run = False
                time_passed = 0
        if trajectory_test:
            angles = trajectory.calculate_step()
            print("step ", trajectory.current_step, "out of ", trajectory.total_steps)
            for i in range(6):
                motors[i].setPosition(angles[i])



    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
