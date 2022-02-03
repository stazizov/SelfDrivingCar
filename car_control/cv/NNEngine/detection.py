import cv2
import numpy as np
import random

CONFIG_PATH = './yolov4-tiny.cfg'
WEIGHTS_PATH = './yolov4-tiny.weights'
CLASSNAMES = [item.strip() for item in open('./coco.names', 'r').readlines()]


class Detector:
    '''
    YOLO detector trained on COCO dataset
    '''

    def __init__(
        self,
        classnames=CLASSNAMES,
        model_cfg=CONFIG_PATH,
        model_weights=WEIGHTS_PATH,
        whT=416,
        confThreshold=0.0,
        nmsThreshold=0.0
    ):
        '''
        Arguments: 

        classnames: list : array of names
        model_cfg : str : path to model config file
        model_weights : str : path to model weights

        Returns:
        YOLO model
        '''

        self.whT = whT
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.classnames = classnames
        self.net = self.define_net(model_cfg, model_weights)

    def define_net(self, model_cfg, model_weights):
        '''
        This function defines yolo detector

        Arguments:

        model_cfg : str : path to model config file
        model_weights : str : path to model weights

        Returns:

        YOLO model
        '''

        net = cv2.dnn.readNetFromDarknet(model_cfg, model_weights)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return net

    def find_objects(self, img, outputs):
        '''
        Applys NMS to yolo outputs

        Arguments: 

        img : np.ndarray : input frame
        outputs : list : YOLO outputs

        Returns:

        array of type [[classname, (x_min, y_min, x_max, y_max)]]
        '''
        hT, wT, cT = img.shape
        bbox = []
        classIds = []
        confs = []
        results = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    w, h = int(detection[2]*wT), int(detection[3]*hT)
                    x, y = int(detection[0] * wT - w /
                               2), int(detection[1] * hT - h / 2)
                    bbox.append([x, y, w, h])
                    classIds.append(classId)
                    confs.append(float(confidence))

        indices = cv2.dnn.NMSBoxes(
            bbox, confs, self.confThreshold, nms_threshold=self.nmsThreshold)

        for i in indices:
            box = bbox[i]
            x, y, w, h = box
            current_coords = (x, y, x + w, y + h)
            current_class = self.classnames[classIds[i]]
            current_conf = confs[i]
            img = self.draw_results(
                img, current_coords, current_conf, current_class)
            results.append([current_class, current_coords])

        return results, img

    def draw_results(self, img, bbox, conf, classname):
        '''
        This function just draw bboxes on given image

        Arguments: 

        img : np.ndarray : input_image
        bbox : np.ndarray : (x_min, y_min, x_max, y_max) coordinates

        Returns:

        Image with rendered bboxes
        '''
        x, y, w, h = bbox
        color = [random.randint(0, 255) for _ in range(3)]
        img = cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        img = cv2.putText(
            img,
            text=f'{classname} : {round(conf, 2)}%',
            org=(x, y + h + 30),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=1,
            color=color,
            lineType=cv2.LINE_4
        )

        return img

    def forward(self, frame):
        '''
        Arguments:
        frame : np.ndarray : input image

        Returns:
        frame : np.ndarray : image with bboxes on it
        bboxes : list : YOLO outputs of type [[classname, (x_min, y_min, x_max, y_max)]]
        '''
        blob = cv2.dnn.blobFromImage(
            frame, 1/255, (self.whT, self.whT), [0, 0, 0], 1, crop=False)
        self.net.setInput(blob)
        layerNames = self.net.getLayerNames()
        outputNames = [layerNames[i - 1]
                       for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(outputNames)
        bboxes, frame = self.find_objects(outputs=outputs, img=frame)
        return bboxes, frame


if __name__ == '__main__':
    image = cv2.imread(filename='test_image.png')
    detector = Detector(CLASSNAMES, CONFIG_PATH, WEIGHTS_PATH)
    bboxes, image = detector.forward(image)
    cv2.imshow("Detection", image)
    cv2.waitKey(0)
