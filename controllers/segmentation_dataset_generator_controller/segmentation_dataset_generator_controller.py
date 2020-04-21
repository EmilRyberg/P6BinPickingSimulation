"""segmentation_dataset_generator_controller controller."""

from controller import Robot
from controller import Connector
from controller import RangeFinder
from controller import Camera
from controller import Supervisor
import random
import math

#robot = Robot()
supervisor = Supervisor()
timestep = 1

#motors = [robot.getMotor("shoulder_pan_joint"), robot.getMotor("shoulder_lift_joint"), robot.getMotor("elbow_joint"),
          #robot.getMotor("wrist_1_joint"), robot.getMotor("wrist_2_joint"), robot.getMotor("wrist_3_joint")]
#motor_sensors = [robot.getPositionSensor("shoulder_pan_joint_sensor"), robot.getPositionSensor("shoulder_lift_joint_sensor"), robot.getPositionSensor("elbow_joint_sensor"),
          #robot.getPositionSensor("wrist_1_joint_sensor"), robot.getPositionSensor("wrist_2_joint_sensor"), robot.getPositionSensor("wrist_3_joint_sensor")]

phone_part_objects = [
    supervisor.getFromDef('Bottom_Cover'),
    supervisor.getFromDef('Bottom_Cover_2'),
    supervisor.getFromDef('White_Cover'),
    supervisor.getFromDef('White_Cover_2'),
    supervisor.getFromDef('Black_Cover'),
    supervisor.getFromDef('Black_Cover_2'),
    supervisor.getFromDef('Blue_Cover'),
    supervisor.getFromDef('Blue_Cover_2'),
    supervisor.getFromDef('PCB'),
    supervisor.getFromDef('PCB_2')
]

translation_fields = [node.getField("translation") for node in phone_part_objects]
rotation_fields = [node.getField("rotation") for node in phone_part_objects]

front_cover_initial_pos = [-0.17, -0.16]
back_cover_initial_pos = [-0.16, -0.16]
pcb_initial_pos = [-0.14, -0.13]
default_rotation = [1, 0, 0, 1.57]

max_movement = [0.08, 0.08]

height = 1.1
height_step = 0.05

for index, (translation_field, rotation_field) in enumerate(zip(translation_fields, rotation_fields)):
    current_position = translation_field.getSFVec3f()
    random_rotation = random.random() * 0.2 + 1.57
    rotation = [1, 0, 0, random_rotation]
    rotation_field.setSFRotation(rotation)
    random_x_shift = random.random() * (max_movement[0] * 2) - max_movement[0]
    random_z_shift = random.random() * (max_movement[1] * 2) - max_movement[1]
    print(current_position)
    if index < 2:
        # back cover
        translation_field.setSFVec3f([back_cover_initial_pos[0] + random_x_shift, height, back_cover_initial_pos[1] + random_z_shift])
    elif 2 <= index < 8:
        # front cover
        translation_field.setSFVec3f([front_cover_initial_pos[0] + random_x_shift, height, front_cover_initial_pos[1] + random_z_shift])
    else:
        translation_field.setSFVec3f([pcb_initial_pos[0] + random_x_shift, height, pcb_initial_pos[1] + random_z_shift])
    height += height_step

for part in phone_part_objects:
    part.resetPhysics()

#for sensor in motor_sensors:
#    sensor.enable(10)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    pass