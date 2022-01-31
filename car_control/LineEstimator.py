import numpy as np
import cv2

class BirdEyeViewTransformer:
    def __init__(self, height, width):
        # targeted rectangle on original image which needs to be transformed
        # tl = [50, 26] # top left
        # tr = [50, height] # top right
        # br = [width, height] # bottom right
        # bl = [0, height] # bottom left

        warped_height = height * 5

        tl = [width//2 - 10, 0]  # top left
        tr = [width//2 + 10, 0]  # top right
        br = [width, height]  # bottom right
        bl = [0, height]  # bottom left

        self.corner_points_array = np.float32([tl, tr, br, bl])

        # Create an array with the parameters (the dimensions) required to build the matrix
        imgTl = [0, 0]
        imgTr = [width, 0]
        imgBr = [width, warped_height]
        imgBl = [0, warped_height]

        self.img_params = np.float32([imgTl, imgTr, imgBr, imgBl])
        self.width = width
        self.height = warped_height

    def __call__(self, image):
        # Compute and return the transformation matrix
        matrix = cv2.getPerspectiveTransform(
            self.corner_points_array, self.img_params)
        img_transformed = cv2.warpPerspective(
            image, matrix, (self.width, self.height))

        return img_transformed


class LineEstimator:
    def __init__(
        self,
        image_height,
        image_width,
        image_channels,
    ):
        self.IMAGE_HEIGHT = image_height
        self.IMAGE_WIDTH = image_width
        self.IMAGE_CHANNELS = image_channels
        self.bird_eye_view_transformer = BirdEyeViewTransformer(
            image_height, image_width)

    def getThresh(self, image):

        if len(image.shape) == 3:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY).astype(np.uint8)

        # image =cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                        #   cv2.THRESH_BINARY_INV, 25, 5)
        # Find Canny edges
        edged = cv2.Canny(image, 30, 200)
        
        # Finding Contours
        # Use a copy of the image e.g. edged.copy()
        # since findContours alters the image
        _, contours, hierarchy = cv2.findContours(edged, 
            cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        # Draw all contours
        # -1 signifies drawing all contours
        image = cv2.drawContours(image, contours, -1, (255, 255, 255), 30)
        image[image < 255] = 0

        return image

    def getBlur(self, image):
        return cv2.GaussianBlur(image, (11, 11), 0)

    def drawWayAndGetCenter(self, image):
        ys, xs = np.nonzero(image)
        indices = list(zip(xs, ys))
        indices = np.array(indices)

        last_y = -1
        avg_arr = []
        res = []
        for (x, y) in indices:
            if y > last_y:
                if len(avg_arr) != 0:
                    res.append([sum(avg_arr) // len(avg_arr), y])
                last_y = y
                avg_arr = []
            avg_arr.append(x)
        
        image = cv2.merge([image,]*3)
        for coords in res:
            image = cv2.circle(image, (coords[0], coords[1]), 1, (0, 255, 0), 2)

        res = [r[0] for r in res[-50:]]
        avg = sum(res) / len(res)

        image = cv2.circle(image, (int(avg), self.IMAGE_HEIGHT - 50), 12, (0, 255, 255), 2)

        half_width = float(self.IMAGE_WIDTH) / 2

        avg = (avg - half_width) / half_width
        
        return image, avg

    def __call__(self, image):
        '''
        Returns coordinate of center of road in range [-1, 1]
        '''

        image = self.bird_eye_view_transformer(image)[-150:, ]
        image = self.getBlur(image)
        image = self.getThresh(image)
        image, center = self.drawWayAndGetCenter(image)
        return image, center