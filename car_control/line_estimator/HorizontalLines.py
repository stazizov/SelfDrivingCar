from matplotlib import image
import matplotlib.pyplot as plt
import numpy as np
import cv2

from PerspectiveTransformation import PerspectiveTransformation
from LaneLines import LaneLines
from Thresholding import Thresholding
from PerspectiveTransformation import PerspectiveTransformation

class HorizontalLines:
    def __init__(self, ):
        '''
        This class trying to detect 
        horizontal stop-line on the road
        '''

        # region of interest : list : [ty, tx, by, bx]
        pass
        
    def get_hist(self, image, kn = 15, threshold = 23):
        '''
        input : binary image
        output : histogram of white pixels distribution
        '''
        plot = cv2.merge([np.ones_like(image)] * 3)
        sums = np.sum(image / 255, axis = 1).astype(np.float32)
        sums = [int(sum(sums[index:index + kn]) / kn) for index in range(len(sums - kn))]
        for index, value in enumerate(sums):
            if value > threshold:
                plot[index, :] *= (0, 0, 255)
            else:
                plot[index, :value] *= (255 - min(value*10, 255), 255 - min(value*10, 255), min(value*10, 255))
        return plot

    def mark_line(self, image, kn = 10, threshold = 23):
        '''
        input : binary image
        output : 3D image with marked road
        '''
        sums = np.sum(image / 255, axis = 1).astype(np.float32)
        sums = [int(sum(sums[index:index + kn]) / kn) for index in range(len(sums - kn))]
        image = cv2.merge([image]*3).astype(np.float32)
        for index, value in enumerate(sums):
            if value > threshold:
                image[index, :] *= (0, 0, 255)
            else:
                image[index, :value] *= (255 - min(value*10, 255), 255 - min(value*10, 255), min(value*10, 255))
        return image
    
    def forward(self, image):
        '''
        inputs: 

        image : np.ndarray

        output : bool : is there stop-line ahead
        '''

        return self.mark_line(image)


if __name__ == "__main__":

    cap = cv2.VideoCapture('test_video.mov')

    img_size = (280, 320)
    num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) 

    HorizontalLinesDetector = HorizontalLines()
    PerspectiveTransformationHelper = PerspectiveTransformation(img_size[::-1])
    ThresholdingHelper = Thresholding()

    for _ in range(num_frames):
        success, frame = cap.read()
        if success:
            frame = cv2.resize(frame, img_size)
            frame = PerspectiveTransformationHelper.forward(frame)
            frame = ThresholdingHelper.forward(frame)
            cv2.imshow('frame', frame)
            frame = HorizontalLinesDetector.forward(frame)
            cv2.imshow('hist', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

        

        