from .api.simulator import SimulatorAPI
import rclpy
from .api.camera import CameraAPI
from .car_control.main import image_process

print("hello buenos")

def main(args=None):
    rclpy.init(args=args)
    sim_api = SimulatorAPI()
    camera = CameraAPI(image_process, sim_api)
    rclpy.spin(camera)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
