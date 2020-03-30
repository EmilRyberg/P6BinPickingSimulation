"""ur_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from kinematics import Kinematics, DHParameters

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

for sensor in motor_sensors:
    sensor.enable(10)
motors[0].setPosition(np.pi / 2)
first_run = True
can_run = False
time_passed = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    if first_run and can_run:
        kin = Kinematics()
        T0W = kin.compute_0_to_W_matrix([motor_sensors[0].getValue(),
                                         motor_sensors[1].getValue(),
                                         motor_sensors[2].getValue(),
                                         motor_sensors[3].getValue(),
                                         motor_sensors[4].getValue(),
                                         motor_sensors[5].getValue()])
        print("T0W: ", T0W)
        first_run = False
    elif not can_run:
        time_passed += timestep
        if time_passed > 1000:
            can_run = True
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
