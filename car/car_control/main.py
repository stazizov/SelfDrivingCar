from .car import Car
from ..api.simulator import SimulatorAPI
from .cv.RoadDetector import RoadDetector

car = Car(
    road_detector=RoadDetector((1280 // 4, 720 // 4)),
    PID_settings=(80, 0, 5),
    max_speed=20,
    min_speed=5
)


def image_process(image, sim_api):
    if not car.sim_api:
        car.sim_api = sim_api

    debug_image = car.follow_road(image)
    sim_api.imshow(debug_image)
