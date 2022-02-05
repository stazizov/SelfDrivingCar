from .car import Car
from ..api.simulator import SimulatorAPI
from .cv.RoadDetector import RoadDetector

road_detector = RoadDetector((320, 320))

car = Car(
    PID_settings=(80, 0, 5),
    max_speed=20,
    min_speed=5
)


def image_process(image, sim_api):
    if not car.sim_api:
        car.sim_api = sim_api

    road_info = road_detector.forward(image)
    car.follow_road(road_info)
    sim_api.imshow(road_info.frame)
