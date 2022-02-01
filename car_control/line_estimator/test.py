import numpy as np
import cv2

from LaneLines import LaneLines
from Thresholding import Thresholding
from PerspectiveTransformation import PerspectiveTransformation
from HorizontalLines import HorizontalLines

def main(video_path, img_size, kn, thresold):
    cap = cv2.VideoCapture(video_path)

    num_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) 
    HorizontalLinesHelper= HorizontalLines(img_size, kn, THRESHOLD)
    PerspectiveTransformationHelper = PerspectiveTransformation(img_size[::-1])
    ThresholdingHelper = Thresholding()
    LaneLinesHelper = LaneLines()

    for _ in range(num_frames):
        success, input_frame = cap.read()
        if success:
            input_frame = cv2.resize(input_frame, img_size)
            # bird's eye view
            frame = PerspectiveTransformationHelper.forward(input_frame.copy())
            # Thresholding to get binary image of road
            frame = ThresholdingHelper.forward(frame)
            # binary mask of horizontal lines and distance to this lines
            mask, distance = HorizontalLinesHelper.forward(frame)
            # Estimate lane lines
            frame = LaneLinesHelper.forward(frame)[0]

            # backward perspective transformation
            mask = PerspectiveTransformationHelper.backward(mask)
            mask = cv2.merge([mask]*3) * (255,0,255)
            mask = mask.astype(np.uint8)

            # backward frame transformation 
            frame = PerspectiveTransformationHelper.backward(frame)
            
            # add colored horizontal mask
            frame = np.bitwise_not(mask) / 255 * frame + mask * (255,0,255)
            frame= frame.astype(np.uint8)
            frame = cv2.addWeighted(input_frame , 1, frame, 0.5, 0)

            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break
        
if __name__ == "__main__":

    IMG_SIZE = (320, 320)
    KN = 15
    THRESHOLD = 20
    VIDEO_PATH = 'test_video.mov'

    main(VIDEO_PATH, IMG_SIZE, KN, THRESHOLD)