import numpy as np
import cv2
from .Thresholding import *
from .PerspectiveTransformation import *
from .LaneLines import *
from .HorizontalLines import HorizontalLines
from .RoadInfo import RoadInfo


class RoadDetector:
    def __init__(self, img_size):
        """ Init Application"""
        self.img_size = img_size
        self.HorizontalLinesHelper = HorizontalLines(img_size)
        self.PerspectiveTransformationHelper = PerspectiveTransformation(
            img_size[::-1])
        self.ThresholdingHelper = Thresholding()
        self.LaneLinesHelper = LaneLines()

    def forward(self, input_frame):
        '''
        Arguments:

        img : np.ndarray : input image

        Returns:
        img(np.ndarray) output image with visualization of entire computation processes 
        pos: float : deviation from the center of the road
        curveness : float : lines curveness
        '''
        input_frame = cv2.resize(input_frame, self.img_size)
        # bird's eye view
        frame = self.PerspectiveTransformationHelper.forward(
            input_frame.copy())
        # Thresholding to get binary image of road
        frame = self.ThresholdingHelper.forward(frame)
        # binary mask of horizontal lines and distance to this lines
        mask, distance = self.HorizontalLinesHelper.forward(frame)
        # Estimate lane lines
        frame, pos, lR, rR = self.LaneLinesHelper.forward(frame)

        # backward perspective transformation
        mask = self.PerspectiveTransformationHelper.backward(mask)
        mask = cv2.merge([mask]*3) * (255, 0, 255)
        mask = mask.astype(np.uint8)

        # backward frame transformation
        frame = self.PerspectiveTransformationHelper.backward(frame)

        # add colored horizontal mask
        frame = np.bitwise_not(mask) / 255 * frame + mask * (255, 0, 255)
        frame = frame.astype(np.uint8)

        frame = cv2.addWeighted(input_frame, 1, frame, 0.5, 0)

        return RoadInfo(
            frame=frame,
            position=pos,
            curveness=(lR, rR),
            stop_distance=distance
        )
