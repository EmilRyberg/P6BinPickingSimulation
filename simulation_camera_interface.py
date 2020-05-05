from controllers.ur_controller.P6BinPicking.camera_interface import CameraInterface


class SimulationCamera(CameraInterface):
    def __init__(self, simulation_connector):
        self.connector = simulation_connector
        assert simulation_connector != None, "simulation_connector_instance cannot be empty in simulation mode"

    def get_image(self):
        image = self.connector.get_image()
        return image

    def get_depth(self):
        depth = self.connector.get_depth()
        return depth