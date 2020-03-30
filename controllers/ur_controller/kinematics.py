import numpy as np

class DHParameters:
    def __init__(self, a, d, alpha):
        self.a = a
        self.d = d
        self.alpha = alpha


class Kinematics:
    def __init__(self):
        self.joint1_dh = DHParameters(0, 0.1625, 0)
        self.joint2_dh = DHParameters(0, 0, np.pi / 2)
        self.joint3_dh = DHParameters(-0.425, 0, 0)
        self.joint4_dh = DHParameters(-0.39225, 0.1333, 0)
        self.joint5_dh = DHParameters(0, 0.0997, np.pi / 2)
        self.joint6_dh = DHParameters(0, 0.0996, -np.pi / 2)

    def compute_transformation_matrix(self, theta, dh_params):
        c = np.cos(theta)
        s = np.sin(theta)
        ca = np.cos(dh_params.alpha)
        sa = np.sin(dh_params.alpha)
        A = [[c, -s, 0, dh_params.a],
             [s*ca, c*ca, -sa, -sa*dh_params.d],
             [s*sa, c*sa, ca, ca*dh_params.d],
             [0, 0, 0, 1]]
        A = np.array(A)
        return A

    def compute_0_to_W_matrix(self, thetas):
        RX = np.array([[1, 0, 0, 0],
                        [0, np.cos(-np.pi / 2), -np.sin(-np.pi / 2), 0],
                        [0, np.sin(-np.pi / 2), np.cos(-np.pi / 2), 0],
                        [0, 0, 0, 1]])
        RZ = np.array([[np.cos(np.pi), -np.sin(np.pi), 0, 0],
                        [np.sin(np.pi), np.cos(np.pi), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        TWORLD0 = np.matmul(RX, RZ)
        T01 = self.compute_transformation_matrix(thetas[0], self.joint1_dh)
        T12 = self.compute_transformation_matrix(thetas[1], self.joint2_dh)
        T23 = self.compute_transformation_matrix(thetas[2], self.joint3_dh)
        T34 = self.compute_transformation_matrix(thetas[3], self.joint4_dh)
        T45 = self.compute_transformation_matrix(thetas[4], self.joint5_dh)
        T5W = self.compute_transformation_matrix(thetas[5], self.joint6_dh)
        T0W = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(TWORLD0, T01), T12), T23), T34), T45), T5W)
        return T0W