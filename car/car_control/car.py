from enum import Enum
import cv2
from .cv.RoadDetector import RoadDetector
from simple_pid import PID
import asyncio


class TurnDirection(Enum):
    Left = 1
    Right = 2


class Car:
    '''
    Main Car conrol class
    '''

    def __init__(self,
                 PID_settings,
                 max_speed,
                 min_speed
                 ):
        self.sim_api = None

        self.PID = PID(*PID_settings, setpoint=0)
        self.PID.output_limits = (-90, 90)

        self.last_error = 0
        self.curveness = None

        self.MAX_SPEED = max_speed
        self.MIN_SPEED = min_speed

        self.RED_STOP_DISTANCE = 50
        self.YELLOW_STOP_DISTANCE = 200

    def follow_road(self, road_info):
        angle = self.PID(self.last_error)
        speed = self.MAX_SPEED

        if self.curveness is not None and min(self.curveness) < 100:
            speed = self.MIN_SPEED

        print(speed, angle)
        self.sim_api.go(speed, angle)

        self.last_error = road_info.position
        self.curveness = road_info.curveness

    async def stop(self, delay=2):
        await asyncio.sleep(delay)

    def turn(self, direction):
        self.last_error = 0
        self.curveness = None
        # TODO: Implement turn logic
        if direction == TurnDirection.Left:
            pass
        elif direction == TurnDirection.Right:
            pass

    def decide(self, road_info):

        if road_info.stop_distance is None:
            self.follow_road(road_info)
        elif road_info.stop_distance >= self.RED_STOP_DISTANCE:
            self.stop()
