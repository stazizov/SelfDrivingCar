from enum import Enum
from simple_pid import PID
from datetime import datetime


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

        self.turn_start_steps = 0

        self.TURN_RIGHT_STEPS = (10, 72, 77)
        self.TURN_RIGHT_ANGLE = 34

        self.TURN_LEFT_ANGLE = -34
        self.TURN_LEFT_STEPS = (25, 100, 105)

        self.turning = False

        self.right_encoder_value = 0
        self.left_encoder_value = 0

    def set_api(self, sim_api):
        self.sim_api = sim_api
        self.sim_api.encoder_handler = self.encoder_handler

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
        if not self.turning:
            self.turning = True
            self.last_error = 0
            self.curveness = None
            self.turn_start_time = datetime.now()
            # TODO: Implement turn logic

            if direction == TurnDirection.Left:
                self.turn_start_steps = self.left_encoder_value
            elif direction == TurnDirection.Right:
                self.turn_start_steps = self.right_encoder_value
        else:
            if direction == TurnDirection.Right:
                delta = self.right_encoder_value - self.turn_start_steps
                steps = self.TURN_RIGHT_STEPS
                angle = self.TURN_RIGHT_ANGLE
            else:
                delta = self.left_encoder_value - self.turn_start_steps
                steps = self.TURN_LEFT_STEPS
                angle = self.TURN_LEFT_ANGLE

            if delta <= steps[0]:
                self.sim_api.go(10, 0)
            elif delta <= steps[1]:
                self.sim_api.go(10, angle)
            elif delta <= steps[2]:
                self.sim_api.go(10, 0)
            else:
                self.turning = False
                self.turn_start_time = None

    def encoder_handler(self, left, right):
        self.left_encoder_value = left
        self.right_encoder_value = right
