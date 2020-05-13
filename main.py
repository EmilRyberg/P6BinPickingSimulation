import sys
#sys.path.append('controllers/ur_controller')
#sys.path.append('controllers/ur_controller/P6BinPicking')

from controllers.ur_controller.P6BinPicking.controller.controller import Controller
from simulation_connector import SimulationConnector
from simulation_camera_interface import SimulationCamera


if __name__ == "__main__":
    connector = SimulationConnector(2000)
    camera = SimulationCamera(connector)
    controller = Controller(connector, camera, "model_final_sim.pth")
    controller.main_flow(1, True)