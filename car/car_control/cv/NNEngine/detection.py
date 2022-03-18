import cv2
import numpy as np
import random
import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

CONFIG_PATH = '/home/dev_ws/src/car/car/car_control/cv/NNEngine/yolov4-tiny.cfg'
WEIGHTS_PATH = '/home/dev_ws/src/car/car/car_control/cv/NNEngine/yolov4-tiny.weights'
CLASSNAMES_PATH = '/home/dev_ws/src/car/car/car_control/cv/NNEngine/obj.names'

class DetectedObject:
    def __init__(self, name, coords):
        self.name = name
        self.coords = coords

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

    '''

    def __init__(
        self,
        classnames_path=CLASSNAMES_PATH,
        model_cfg=CONFIG_PATH,
        model_weights=WEIGHTS_PATH,
        whT=416,
        confThreshold=0.7,
        nmsThreshold=0.7,
        desired_classes=None
    ):
        # Detector Parameters
        self.whT = whT
        self.confThreshold = confThreshold
        self.nmsThreshold = nmsThreshold
        self.classnames = [item.strip()
                           for item in open(classnames_path, 'r').readlines()]
        self.net = self.define_net(model_cfg, model_weights)
        # we define this list because coco consists 80 classes but we need only 3
        self.desired_classes = desired_classes if desired_classes is not None else self.classnames
        self.colors = dict(
            zip(
                self.desired_classes,
                [[random.randint(0, 255) for _ in range(3)]
                 for _ in self.desired_classes]
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
            array of type DetectedObject
        '''

        hT, wT, _ = img.shape
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

                    center_x = int(detection[0] * wT)
                    center_y = int(detection[1] * hT)
                    w = int(detection[2] * wT)
                    h = int(detection[3] * hT)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    bbox.append([x, y, w, h])
                    classIds.append(classId)
                    confs.append(float(confidence))

        indices = cv2.dnn.NMSBoxes(
            bbox, confs, self.confThreshold, nms_threshold=self.nmsThreshold)

        for i in indices:
            box = bbox[i]
            x, y, w, h = box
            current_coords = (x, y, w, h)
            current_class = self.classnames[classIds[i]]
            current_conf = confs[i]
            if current_class in self.desired_classes:
                results.append(DetectedObject(current_class, current_coords))
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
        print(bbox)
        x, y, w, h = [max(coord, 0) for coord in bbox]
        x1, y1, x2, y2 = x, y, x+w, y+h
        color = self.colors[classname]
        label = f'{classname}: {round(conf, 2)}%'

        # For bounding box
        img = cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)

        # For the text background
        # Finds space required by the text so that we can put a background with that amount of width.
        (w, h), _ = cv2.getTextSize(
            label, cv2.LINE_AA, 0.6, 2)

        # Prints the text.
        img = cv2.rectangle(img, (x1, y1 - 20), (x1 + w, y1), color, -1)
        img = cv2.putText(img, label, (x1, y1 - 5),
                          cv2.LINE_AA, 0.6, (255, 255, 255), 2)

        return img

    def forward(self, frame):
        '''
        Arguments:
            frame(np.ndarray): input image

        Returns:
            frame(np.ndarray): image with bboxes on it
            bboxes(list): YOLO outputs of type list[DetectedObject]
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
