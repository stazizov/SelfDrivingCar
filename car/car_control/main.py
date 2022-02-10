from .car import Car, TurnDirection
from .cv.RoadDetector import RoadDetector
from .cv.NNEngine.detection import Detector
from datetime import datetime

road_detector = RoadDetector((320, 320))
detector = Detector()

car = Car(
    PID_settings=(80, 0, 5),
    max_speed=20,
    min_speed=5
)

STOP_DISTANCE = 80

detected_signs = []
direction = TurnDirection.Left


def image_process(image, sim_api):
    global last_stop_time, detected_signs, direction
    if not car.sim_api:
        car.set_api(sim_api)

    road_info = road_detector.forward(image)
    frame = None

    if not car.turning:
        if road_info.stop_distance is not None and road_info.stop_distance <= 200:
            if len(detected_signs) == 0:
                detected_signs, frame = detector.forward(image)

        if road_info.stop_distance is not None and road_info.stop_distance <= STOP_DISTANCE:
            if len(detected_signs) > 0:
                name = detected_signs[0].name
                print(name)
                if name == "parking" or name == "crosswalk":
                    direction = TurnDirection.Right
                    car.turn(direction)
                    print("TURNING RIGHT")
                    detected_signs = []
                elif name == "stop":
                    direction = TurnDirection.Left
                    car.turn(direction)
                    print("TURNING LEFT")
                    detected_signs = []
        elif road_info.stop_distance is None or road_info.stop_distance >= 150:
            car.follow_road(road_info)
    else:
        car.turn(direction)

    if frame is not None:
        sim_api.imshow(frame)
