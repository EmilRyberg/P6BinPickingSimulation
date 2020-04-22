"""segmentation_dataset_generator_controller controller."""

from controller import Robot
from controller import Connector
from controller import RangeFinder
from controller import Camera
from controller import Supervisor
import random
import socket
import struct
import pickle
import numpy as np
import math
import cv2

#robot = Robot()
supervisor = Supervisor()
connector = None
timestep = 5

conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

motors = [supervisor.getMotor("shoulder_pan_joint"), supervisor.getMotor("shoulder_lift_joint"), supervisor.getMotor("elbow_joint"),
          supervisor.getMotor("wrist_1_joint"), supervisor.getMotor("wrist_2_joint"), supervisor.getMotor("wrist_3_joint")]
#motor_sensors = [robot.getPositionSensor("shoulder_pan_joint_sensor"), robot.getPositionSensor("shoulder_lift_joint_sensor"), robot.getPositionSensor("elbow_joint_sensor"),
          #robot.getPositionSensor("wrist_1_joint_sensor"), robot.getPositionSensor("wrist_2_joint_sensor"), robot.getPositionSensor("wrist_3_joint_sensor")]

camera = supervisor.getCamera('cameraRGB')
depth_camera = supervisor.getRangeFinder('cameraDepth')
ur_node = supervisor.getFromDef('UR5')
camera_transform_node = ur_node.getField('children').getMFNode(0)
camera_node = camera_transform_node.getField('children').getMFNode(1)
camera.enable(4)
#depth_camera.enable(35)

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

translation_fields = [node.getField('translation') for node in phone_part_objects]
rotation_fields = [node.getField('rotation') for node in phone_part_objects]
front_cover_initial_pos = [-0.17, -0.16]
back_cover_initial_pos = [-0.16, -0.16]
pcb_initial_pos = [-0.14, -0.13]
default_rotation = [1, 0, 0, 1.57]
max_movement = [0.08, 0.08]


def randomize_phone_parts():
    height = 1.1
    height_step = 0.05

    for index, (translation_field, rotation_field) in enumerate(zip(translation_fields, rotation_fields)):
        current_position = translation_field.getSFVec3f()
        random_rotation = random.random() * 0.2 + 1.57
        rotation = [1, 0, 0, random_rotation]
        rotation_field.setSFRotation(rotation)
        random_x_shift = random.random() * (max_movement[0] * 2) - max_movement[0]
        random_z_shift = random.random() * (max_movement[1] * 2) - max_movement[1]
        if index < 2:
            # back cover
            translation_field.setSFVec3f(
                [back_cover_initial_pos[0] + random_x_shift, height, back_cover_initial_pos[1] + random_z_shift])
        elif 2 <= index < 8:
            # front cover
            translation_field.setSFVec3f(
                [front_cover_initial_pos[0] + random_x_shift, height, front_cover_initial_pos[1] + random_z_shift])
        else:
            translation_field.setSFVec3f(
                [pcb_initial_pos[0] + random_x_shift, height, pcb_initial_pos[1] + random_z_shift])
        height += height_step

    for part in phone_part_objects:
        part.resetPhysics()


def set_part_only_visible(ii):
    for i, part in enumerate(phone_part_objects):
        if i == ii:
            continue
        part.setVisibility(camera_node, False)


def toggle_visibility_for_all_parts(visible):
    for part in phone_part_objects:
        part.setVisibility(camera_node, visible)


def transform_image(img_array):
    np_img = np.array(img_array, dtype=np.uint8)
    np_img = np_img.transpose((1, 0, 2))
    np_img_bgr = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
    return np_img_bgr

randomize_phone_parts()

wait_time = 3 # seconds
last_run_time = supervisor.getTime()
take_image = True
is_first_run = True
images_to_take = 1
image_index = 0
supervisor.step(timestep)
toggle_visibility_for_all_parts(False)
supervisor.step(timestep)
background_img = transform_image(camera.getImageArray())
cv2.imwrite('background.png', background_img)
toggle_visibility_for_all_parts(True)

while supervisor.step(timestep) != -1:
    if supervisor.getTime() - last_run_time >= wait_time and image_index < images_to_take:
        for index in range(0, len(phone_part_objects)):
            set_part_only_visible(index)
            supervisor.step(timestep)
            image = transform_image(camera.getImageArray())
            image_subtracted = image - background_img
            cv2.imwrite('img' + str(index) + '.png', image_subtracted)
            toggle_visibility_for_all_parts(True)
        last_run_time = supervisor.getTime()
        image_index += 1
    elif image_index >= images_to_take:
        break
