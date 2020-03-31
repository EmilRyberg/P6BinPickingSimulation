import numpy as np
import math

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


class ForwardKinematics(Kinematics):
    def compute_0_to_6_matrix(self, thetas):
        T01 = self.compute_transformation_matrix(thetas[0], self.joint1_dh)
        T12 = self.compute_transformation_matrix(thetas[1], self.joint2_dh)
        T23 = self.compute_transformation_matrix(thetas[2], self.joint3_dh)
        T34 = self.compute_transformation_matrix(thetas[3], self.joint4_dh)
        T45 = self.compute_transformation_matrix(thetas[4], self.joint5_dh)
        T56 = self.compute_transformation_matrix(thetas[5], self.joint6_dh)
        T06 = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(T01, T12), T23), T34), T45), T56)
        return T06

    def computer_base_to_6_matrix(self, thetas):
        RX = np.array([[1, 0, 0, 0],
                        [0, np.cos(-np.pi / 2), -np.sin(-np.pi / 2), 0],
                        [0, np.sin(-np.pi / 2), np.cos(-np.pi / 2), 0],
                        [0, 0, 0, 1]])
        RZ = np.array([[np.cos(np.pi), -np.sin(np.pi), 0, 0],
                        [np.sin(np.pi), np.cos(np.pi), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        TWORLD0 = np.matmul(RX, RZ)
        T06 = np.matmul(TWORLD0, self.compute_0_to_6_matrix(thetas))

    def convert_T06_to_TB6(self, T06):
        RX = np.array([[1, 0, 0, 0],
                        [0, np.cos(-np.pi / 2), -np.sin(-np.pi / 2), 0],
                        [0, np.sin(-np.pi / 2), np.cos(-np.pi / 2), 0],
                        [0, 0, 0, 1]])
        RZ = np.array([[np.cos(np.pi), -np.sin(np.pi), 0, 0],
                        [np.sin(np.pi), np.cos(np.pi), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
        TWORLD0 = np.matmul(RX, RZ)
        return np.matmul(TWORLD0, T06)


class InverseKinematicsSpecificSolution:
    def __init__(self):
        self.is_valid_solution = True
        self.thetas = []


class InverseKinematicsShoulderSolution:
    def __init__(self):
        self.is_valid_solution = True
        self.solution_elbow_up = InverseKinematicsSpecificSolution()
        self.solution_elbow_down = InverseKinematicsSpecificSolution()


class InverseKinematicsSolution:
    def __init__(self):
        self.solution_shoulder_left = InverseKinematicsShoulderSolution()
        self.solution_shoulder_left.is_valid_solution = False
        self.solution_right_shoulder = InverseKinematicsShoulderSolution()
        self.solution_right_shoulder.is_valid_solution = False


class InverseKinematics(Kinematics):
    def __init__(self):
        super().__init__()
        self.forward_kinematics = ForwardKinematics()
        #self.T06_zero = self.forward_kinematics.compute_0_to_6_matrix([0, 0, 0, 0, 0, 0])

    def __compute_solution_for_theta_1(self, T06, theta_1):
        shoulder_solution = InverseKinematicsShoulderSolution()

        # Theta 5
        P06 = T06[:, 3]
        print("P06: ", P06)
        theta_5_1 = None
        theta_5_2 = None

        theta_5_cos = (P06[0] * math.sin(theta_1) - P06[1] * np.cos(
            theta_1) - self.joint4_dh.d) / self.joint6_dh.d
        if 0 <= theta_5_cos <= 1:
            theta_5_1 = math.acos(theta_5_cos)
            theta_5_2 = -math.acos(theta_5_cos)
            # elif 0 <= theta_5_cos - 1 <= 0.1:
            #     theta_5_1_1 = 0
            #     theta_5_1_2 = 0
            # elif 0 <= theta_5_cos + 1 <= 0.9:
            #     theta_5_1_1 = 1
            #     theta_5_1_2 = -1

        #print(f"Theta5: {theta_5_1_1}, {theta_5_1_2}, {theta_5_2_1}, {theta_5_2_2}")

        # Theta 6
        T60 = np.linalg.inv(T06)
        X60 = T60[:, 0]
        Y60 = T60[:, 1]

        theta_5 = theta_5_1 if theta_5_1 != 0 else theta_5_2
        if theta_5 == 0 or theta_5 is None:
            shoulder_solution.is_valid_solution = False
            return shoulder_solution

        theta_6_cos = (X60[0] * math.sin(theta_1) - Y60[0] * math.cos(theta_1)) / math.sin(
            theta_5)  # only using one of the theta 5's for now..
        theta_6_sin = (-X60[1] * math.sin(theta_1) + Y60[1] * math.cos(theta_1)) / math.sin(
            theta_5)  # only using one of the theta 5's for now..
        theta_6 = math.atan2(theta_6_sin, theta_6_cos)

        #print(f"Theta6: {theta_6_1}, {theta_6_2}")

        # Theta 3
        theta_3_up = None
        theta_3_down = None

        T01 = self.compute_transformation_matrix(theta_1, self.joint1_dh)
        T45 = self.compute_transformation_matrix(theta_5, self.joint5_dh)
        T56 = self.compute_transformation_matrix(theta_6, self.joint6_dh)
        T46 = np.matmul(T45, T56)
        T64 = np.linalg.inv(T46)
        T10 = np.linalg.inv(T01)
        T14 = np.matmul(np.matmul(T10, T06), T64)
        P14 = T14[:, 3]
        theta_3_cos = (math.sqrt(
            P14[0] ** 2 + P14[2] ** 2) ** 2 - self.joint3_dh.a ** 2 - self.joint4_dh.a ** 2) / (
                                  2 * (-self.joint3_dh.a) * (-self.joint4_dh.a))
        if not -1 <= theta_3_cos <= 1:
            shoulder_solution.is_valid_solution = False
            return shoulder_solution

        theta_3_up = math.acos(theta_3_cos)
        theta_3_down = -math.acos(theta_3_cos)

        shoulder_solution.solution_elbow_up = self.__compute_specific_solution(P14, T06, theta_1, theta_3_up, theta_5, theta_6)
        shoulder_solution.solution_elbow_down = self.__compute_specific_solution(P14, T06, theta_1, theta_3_down, theta_5, theta_6)

        return shoulder_solution

        # if not math.isnan(theta_1_2) and theta_6_2 is not None:
        #     T01 = self.compute_transformation_matrix(theta_1_2, self.joint1_dh)
        #     T45 = self.compute_transformation_matrix(theta_5_2_1, self.joint5_dh)
        #     T56 = self.compute_transformation_matrix(theta_6_2, self.joint6_dh)
        #     T46 = np.matmul(T45, T56)
        #     T64 = np.linalg.inv(T46)
        #     T10 = np.linalg.inv(T01)
        #     T14 = np.matmul(np.matmul(T10, T06), T64)
        #     P14 = T14[:, 3]
        #     theta_3_cos = (math.sqrt(
        #         P14[0] ** 2 + P14[2] ** 2) ** 2 - self.joint3_dh.a ** 2 - self.joint4_dh.a ** 2) / (
        #                               2 * (-self.joint3_dh.a) * (-self.joint4_dh.a))
        #     theta_3_2 = math.acos(theta_3_cos)

        #print(f"Theta3: {theta_3_1}, {theta_3_2}")

    def __compute_specific_solution(self, P14, T06, theta_1, theta_3, theta_5, theta_6):
        specific_solution = InverseKinematicsSpecificSolution()

        phi_1 = math.atan2(-P14[2], -P14[0])
        phi_2 = math.asin((-self.joint4_dh.a * math.sin(theta_3)) / math.sqrt(P14[0]**2 + P14[2]**2))
        theta_2 = phi_1 - phi_2

        T01 = self.compute_transformation_matrix(theta_1, self.joint1_dh)
        T12 = self.compute_transformation_matrix(theta_2, self.joint2_dh)
        T23 = self.compute_transformation_matrix(theta_3, self.joint3_dh)
        T45 = self.compute_transformation_matrix(theta_5, self.joint5_dh)
        T56 = self.compute_transformation_matrix(theta_6, self.joint6_dh)
        T03 = np.matmul(np.matmul(T01, T12), T23)
        T30 = np.linalg.inv(T03)
        T46 = np.matmul(T45, T56)
        T64 = np.linalg.inv(T46)
        T34 = np.matmul(np.matmul(T30, T06), T64)
        X34 = T34[:, 0]

        theta_4 = math.atan2(X34[1], X34[0])

        specific_solution.thetas = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

        return specific_solution


    def compute_joint_angles(self, T06):
        solution = InverseKinematicsSolution()
        #Theta 1
        P05 = np.dot(T06, [0, 0, -self.joint6_dh.d, 1])
        print("P05: ", P05)
        phi_1 = math.atan2(P05[1], P05[0])
        phi_2_cos = self.joint4_dh.d / math.sqrt(P05[0]**2 + P05[1]**2)
        print("phi 2 cos: ", phi_2_cos)
        phi_2 = math.acos(phi_2_cos)
        theta_1_1 = phi_1 + phi_2 + (np.pi / 2)
        theta_1_2 = phi_1 - phi_2 + (np.pi / 2)
        print(f"Theta1: {theta_1_1} and {theta_1_2}")

        if not math.isnan(theta_1_1):
            solution.solution_shoulder_left = self.__compute_solution_for_theta_1(T06, theta_1_1)
        if not math.isnan(theta_1_2):
            solution.solution_right_shoulder = self.__compute_solution_for_theta_1(T06, theta_1_2)

        print("Inverse Solutions:")
        if solution.solution_shoulder_left.is_valid_solution and solution.solution_shoulder_left.solution_elbow_up.is_valid_solution:
            print(f"Shoulder left, elbow up: {solution.solution_shoulder_left.solution_elbow_up.thetas}")
        if solution.solution_shoulder_left.is_valid_solution and solution.solution_shoulder_left.solution_elbow_down.is_valid_solution:
            print(f"Shoulder left, elbow down: {solution.solution_shoulder_left.solution_elbow_down.thetas}")
        if solution.solution_right_shoulder.is_valid_solution and solution.solution_right_shoulder.solution_elbow_up.is_valid_solution:
            print(f"Shoulder right, elbow up: {solution.solution_right_shoulder.solution_elbow_up.thetas}")
        if solution.solution_right_shoulder.is_valid_solution and solution.solution_right_shoulder.solution_elbow_down.is_valid_solution:
            print(f"Shoulder right, elbow down: {solution.solution_right_shoulder.solution_elbow_down.thetas}")

        return solution
