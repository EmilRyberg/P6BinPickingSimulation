import numpy as np
from trajectory import Trajectory

class Move:
    def __init__(self, motors, motor_sensors, timestep):
        self.motors = motors
        self.motor_sensors = motor_sensors
        self.timestep = timestep
        self.trajectory = Trajectory(self.motor_sensors, self.timestep)
        self.done = False

    def movej(self, angles, speed=1):
        for i in range(6):
            self.motors[i].setSpeed(speed)
            self.motors[i].setPosition(angles[i])