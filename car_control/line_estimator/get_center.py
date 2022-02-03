import numpy as np
import cv2
from Thresholding import *
from PerspectiveTransformation import *
from LaneLines import *

class FindLaneLines:
    def __init__(self, img_size):
        """ Init Application"""
        self.thresholding = Thresholding()
        self.transform = PerspectiveTransformation(img_size)
        self.lanelines = LaneLines()

    def forward(self, img):
        out_img = np.copy(img)
        img = self.transform.forward(img)
        img = self.thresholding.forward(img)
        img2, pos, lR, rR = self.lanelines.forward(img)
        img = self.transform.backward(img2)

        out_img = cv2.addWeighted(out_img, 1, img, 0.6, 0)
        return img2, pos, lR, rR
            

IMG_SIZE = (320, 180)
findLaneLines = FindLaneLines(
        img_size=IMG_SIZE
    )

def get_center(img):
    frame, pos, lR, rR = findLaneLines.forward(img)

    return pos, lR, rR, frame

def main():
    findLaneLines = FindLaneLines(
        img_size=IMG_SIZE
    )
    # findLaneLines.process_image('./test_images/challenge_video_frame_1.jpg', 'output_image.jpg')

    capture = cv2.VideoCapture('test_video.mov')
    while capture.isOpened():

        frame = capture.read()[1]
        frame = cv2.resize(frame, IMG_SIZE)

        frame, pos, lR, rR = findLaneLines.forward(frame)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break



if __name__ == "__main__":
    main()