from enum import Enum
import cv2
from .cv.RoadDetector import RoadDetector
from simple_pid import PID


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

    def follow_road(self, road_info):
        angle = self.PID(self.last_error)
        speed = self.MAX_SPEED

        if self.curveness is not None and min(self.curveness) < 100:
            speed = self.MIN_SPEED

        self.sim_api.go(speed, angle)

        self.last_error = road_info.position
        self.curveness = road_info.curveness

    def stop(self):
        self.sim_api.go(0, 0)

    def turn(self, direction):
        self.last_error = 0
        self.curveness = None
        # TODO: Implement turn logic
        if direction == TurnDirection.Left:
            pass
        elif direction == TurnDirection.Right:
            pass      
