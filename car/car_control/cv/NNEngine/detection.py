import cv2
import numpy as np
import random

CONFIG_PATH = './yolov4-tiny.cfg'
WEIGHTS_PATH = './yolov4-tiny.weights'
CLASSNAMES_PATH = './coco.names'


class Detector:
    '''
    Description:
        YOLO detector trained on COCO dataset

    Arguments: 
        classnames(list): array of names
        model_cfg(str): path to model config file
        model_weights(str): path to model weights

    Returns:
        YOLO model
        '

    '''
    def __init__(
        self,
        classnames_path=CLASSNAMES_PATH,
        model_cfg=CONFIG_PATH,
        model_weights=WEIGHTS_PATH,
        whT = 416,
        confThreshold=0.0,
        nmsThreshold=0.0,
        desired_classes = ['traffic light', 'stop sign', 'person']
    ):
        # Detector Parameters
        self.whT = whT
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.classnames = [item.strip() for item in open(classnames_path, 'r').readlines()]
        self.net = self.define_net(model_cfg, model_weights)
        # we define this list because coco consists 80 classes but we need only 3
        self.desired_classes = desired_classes
        self.colors = dict(
            zip(
                self.desired_classes, 
                [[random.randint(0, 255) for _ in range(3)] for _ in self.desired_classes]
                )
            )

    def define_net(self, model_cfg, model_weights):
        '''
        Description:
            This function defines yolo detector

        Arguments:
            model_cfg(str): path to model config file
            model_weights(str): path to model weights

        Returns:
            YOLO model
        '''

        net = cv2.dnn.readNetFromDarknet(model_cfg, model_weights)
        net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        return net

    def find_objects(self, img, outputs):
        '''
        Description:
            Applys NMS to yolo outputs

        Arguments: 
            img(np.ndarray) : input frame
            outputs(list) : YOLO outputs

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
            if current_class in self.desired_classes:
                results.append([current_class, current_coords])
                img = self.draw_results(
                    img, current_coords, current_conf, current_class)
        return results, img

    def draw_results(self, img, bbox, conf, classname):
        '''
        Description:
            This function just draw bboxes on given image

        Arguments: 
            img(np.ndarray): input_image
            bbox(np.ndarray): (x_min, y_min, x_max, y_max) coordinates

        Returns:
            Image with rendered bboxes
        '''
        x, y, w, h = [max(coord, 0) for coord in bbox]
        color = self.colors[classname]
        img = cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
        img = cv2.putText(
            img,
            text=f'{classname} : {round(conf, 2)}%',
            org=(x, y+30),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.5,
            color=color,
            lineType=1
        )

        return img

    def forward(self, frame):
        '''
        Arguments:
            frame(np.ndarray): input image

        Returns:
            frame(np.ndarray): image with bboxes on it
            bboxes(list): YOLO outputs of type [[classname, (x_min, y_min, x_max, y_max)]]
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
