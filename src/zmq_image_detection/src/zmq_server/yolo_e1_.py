from __future__ import division

from yolo_e1.models import Darknet
from yolo_e1.utils.utils import rescale_boxes, non_max_suppression, load_classes, display
from yolo_e1.utils.datasets import pad_to_square, resize


import time
import datetime
import numpy as np


from PIL import Image

import torch
from torch.autograd import Variable
import torchvision.transforms as transforms

from config import HEIGHT, WIDTH


class Yolo:
    def __init__(self,
                 img_size=max(HEIGHT, WIDTH),
                 nms_thres=0.4,
                 conf_thres=0.8,
                 display_result=False,
                 model_def='yolo_e1/config/yolov3.cfg',
                 weights_path='yolo_e1/weights/yolov3.weights',
                 class_path='yolo_e1/data/coco.names'
                 ):
        self.img_size = img_size
        self.nms_thres = nms_thres
        self.conf_thres = conf_thres
        self.display_result = display_result
        self.model_def = model_def
        self.weights_path = weights_path
        self.class_path = class_path
        self.device = torch.device(
            'cuda' if torch.cuda.is_available() else 'cpu')
        # Extracts class labels from file
        self.classes = load_classes(self.class_path)
        self.Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor

        # Set up model
        self.model = Darknet(
            self.model_def, img_size=self.img_size).to(self.device)
        self.model.load_darknet_weights(self.weights_path)
        self.model.eval()  # Set in evaluation mode
        print("Initiate Yolo succeeded!")

    def detect(self, np_array):
        prev_time = time.time()

        img = transforms.ToTensor()(np_array)
        img, _ = pad_to_square(img, 0)
        img = resize(img, self.img_size)
        img.unsqueeze_(0)
        img = Variable(img.type(self.Tensor))

        # Get detections
        with torch.no_grad():
            detections = self.model(img)
            detections = non_max_suppression(
                detections, self.conf_thres, self.nms_thres)
            detections = detections[0]

        # Log progress
        current_time = time.time()
        inference_time = datetime.timedelta(seconds=current_time - prev_time)
        prev_time = current_time
        print('Inference Time: %s' % inference_time)

        result = []

        if detections is not None:
            # Rescale boxes to original image
            detections = rescale_boxes(
                detections, self.img_size, np_array.shape[:2])

            for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
                result.append(
                    {'x': int(x1),
                     'y': int(y1),
                     'width': int(x2 - x1),
                     'height': int(y2 - y1),
                     'score': float((conf + cls_conf) / 2),
                     'label': str(self.classes[int(cls_pred)])
                     }
                )

            if self.display_result:
                display(np_array, detections, self.classes)

        return result


if __name__ == '__main__':
    yolo = Yolo(display_result=True)
    print(yolo.detect(np.array(Image.open('test.png'))))
