from .car import Car, TurnDirection
from .cv.RoadDetector import RoadDetector
from datetime import datetime

road_detector = RoadDetector((320, 320))

car = Car(
    PID_settings=(80, 0, 5),
    max_speed=20,
    min_speed=5
)

STOP_DISTANCE = 200
last_stop_time = None


def image_process(image, sim_api):
    global last_stop_time
    if not car.sim_api:
        car.set_api(sim_api)


    # car.turn(TurnDirection.Left)

    road_info = road_detector.forward(image)
    dt = (datetime.now() -
          last_stop_time).seconds if last_stop_time is not None else None
    can_go = dt is None or dt > 5
    can_stop = dt is None or dt > 20

    if road_info.stop_distance is not None and road_info.stop_distance <= STOP_DISTANCE and can_stop:
        print("STOP")
        car.stop()
        last_stop_time = datetime.now()
    elif can_go:
        # print("RIDE", can_stop)
        car.follow_road(road_info)

    sim_api.imshow(road_info.frame)
