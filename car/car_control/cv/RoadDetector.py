import numpy as np
import cv2
from .Thresholding import *
from .PerspectiveTransformation import *
from .LaneLines import *

class RoadDetector:
    def __init__(self, img_size):
        """ Init Application"""
        self.img_size = img_size
        self.thresholding = Thresholding()
        self.transform = PerspectiveTransformation(img_size)
        self.lanelines = LaneLines()

    def forward(self, img):
        '''
        Arguments:
        
        img : np.ndarray : input image

        Returns:
        img: np.ndarray : output image with visualization of entire computation processes 
        pos: float : deviation from the center of the road
        curveness : float : lines curveness
        '''
        out_img = np.copy(img)
        img = self.transform.forward(img)
        img = self.thresholding.forward(img)
        img2, pos, lR, rR = self.lanelines.forward(img)
        img = self.transform.backward(img2)

        out_img = cv2.addWeighted(out_img, 1, img, 0.6, 0)
        return out_img, pos, (lR, rR)
            
