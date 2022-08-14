from .api.simulator import SimulatorAPI
import rclpy
from .car_control.main import image_process


def main(args=None):
    rclpy.init(args=args)
    sim_api = SimulatorAPI(image_process)
    rclpy.spin(sim_api)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
