try:
    from controllers.ur_controller.kinematics import ForwardKinematics, InverseKinematics, DHParameters, InverseKinematicsSolution, InverseKinematicsSpecificSolution, InverseKinematicsShoulderSolution
except:
    pass
from kinematics import ForwardKinematics, InverseKinematics, DHParameters, InverseKinematicsSolution, InverseKinematicsSpecificSolution, InverseKinematicsShoulderSolution
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

class Trajectory:
    def __init__(self, motors, motor_sensors, timestep):
        self.motors = motors
        self.motor_sensors = motor_sensors
        self.timestep = timestep
        self.joint_angles = [0]*6
        self.fkin = ForwardKinematics()
        self.ikin = InverseKinematics()
        self.rotation_matrix = [[0]*3]*3
        self.starting_rotation = None
        self.starting_cart_pos = None
        self.goal_cart_pos = None
        self.goal_rotation = None
        self.total_steps = 0
        self.current_step = 0
        self.slerp = None
        self.is_done = True

    def calculate_distance(self,x1,y1,z1,x2,y2,z2):
        return np.sqrt((x2-x1)**2*(y2-y1)**2*(z2-z1)**2)

    def get_joint_angles(self):
        for i in range(6):
            self.joint_angles[i] = self.motor_sensors.getValue()
        return self.joint_angles

    def generate_trajectory(self, goal, speed):
        self.goal_cart_pos = np.array([goal[0], goal[1], goal[2]])
        self.goal_rotation = R.from_euler('xyz', [goal[3], goal[4], goal[5]], degrees=False)
        transformation_matrix = self.fkin.compute_0_to_6_matrix(self.get_joint_angles())
        self.starting_cart_pos = np.array([transformation_matrix[0][3], transformation_matrix[1][3], transformation_matrix[2][3]])
        for i in range(3):
            for j in range(3):
                self.rotation_matrix[i][j] = transformation_matrix[i][j]
        self.starting_rotation = R.from_rotvec(rotvec=self.rotation_matrix)
        distance = self.calculate_distance(*self.starting_cart_pos, *self.goal_cart_pos)
        self.total_steps = int((distance/speed) / (self.timestep/1000))
        self.slerp = Slerp([0, 1], [self.starting_rotation, self.goal_rotation])
        self.current_step = 0
        self.is_done = False

    def calculate_step(self):
        if not self.is_done:
            self.current_step += 1
            cart_pos = self.starting_cart_pos + (self.current_step / self.total_steps) * (self.goal_cart_pos - self.starting_cart_pos)
            rotation = self.slerp(self.current_step/self.total_steps)
            r = rotation.as_rotvec()
            transformation_matrix = [[r[0][0], r[0][1], r[0][2], cart_pos[0]],
                                     [r[1][0], r[1][1], r[1][2], cart_pos[1]],
                                     [r[2][0], r[2][1], r[2][2], cart_pos[2]],
                                     [0,0,0,1]]
            if self.current_step == self.total_steps:
                self.is_done = True
            return transformation_matrix
        else:
            raise Exception("the current trajectory is already finished")