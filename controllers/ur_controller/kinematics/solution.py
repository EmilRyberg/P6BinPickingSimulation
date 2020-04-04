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
        self.solution_shoulder_right = InverseKinematicsShoulderSolution()
        self.solution_shoulder_right.is_valid_solution = False