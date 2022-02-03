from car import Car
from api.simulator import SimulatorAPI
from cv.RoadDetector import RoadDetector

sim_api = SimulatorAPI()

car = Car(
    sim_api=sim_api,
    road_detector=RoadDetector((1280 // 4, 720 // 4)),
    PID=(80, 0, 5),
    max_speed=20,
    min_speed=5
)


def image_process(image):
    debug_image = car.follow_road(image)
    sim_api.imshow(debug_image)
