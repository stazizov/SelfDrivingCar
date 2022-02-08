import numpy as np
import cv2


class HorizontalLines:
    def __init__(self, image_size, kn = 15, threshold = 20):
        '''
        This class trying to detect 
        horizontal stop-line on the road

        Arguments:

        nk : int : number of rows to compute mean value 
        threshold : int : threshold for sum of nk rows 
        img_size : tuple : (img_height, img_width)
        '''

        # region of interest : list : [ty, tx, by, bx]
        self.kn = kn
        self.threshold = threshold
        self.image_size = image_size
        
    def get_hist(self, image):
        '''
        input : binary image
        output : histogram of white pixels distribution
        '''
        plot = cv2.merge([np.ones_like(image)] * 3)
        sums = np.sum(image / 255, axis = 1).astype(np.float32)
        sums = [int(sum(sums[index:index + self.kn]) / self.kn) for index in range(len(sums - self.kn))]
        for index, value in enumerate(sums):
            if value > self.threshold:
                plot[index, :] *= (0, 0, 255)
            else:
                plot[index, :value] *= (255 - min(value*10, 255), 255 - min(value*10, 255), min(value*10, 255))
        return plot

    def mark_line(self, image):
        '''
        Arguments : 
        
        image : np.ndarray : array of size (H, W)
        
        Outputs: image of size  (H, W) with marked road
        '''
        mask = np.zeros_like(image)
        sums = np.sum(image / 255, axis = 1).astype(np.float32)
        sums = [int(sum(sums[index:index + self.kn]) / self.kn) for index in range(len(sums - self.kn))]
        
        max_index = 0
        for index, value in enumerate(sums):
            if value > self.threshold:
                mask[index, :] = 255
                if max_index < index:
                    max_index = index

        distance = self.image_size[0] - max_index if max_index else None
        return mask, distance
    
    def forward(self, image):
        '''
        Arguments: 
        image : np.ndarray

        output : tuple : (masked_image, distance)
        '''

        return self.mark_line(image)
        