from __future__ import division

import sys
from os import path
from config import PROJECT_ROOT
sys.path.append(path.join(PROJECT_ROOT, 'src/third_party/yolo'))  # noqa: E402

from models import Darknet
from utils.utils import rescale_boxes, non_max_suppression, load_classes
from utils.datasets import pad_to_square, resize
import time
import random
import datetime
import math
import numpy as np
from PIL import Image
import torch
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import NullLocator
from torch.autograd import Variable
import torchvision.transforms as transforms


class Yolo:
    def __init__(self,
                 nms_thres=0.4,
                 conf_thres=0.8,
                 model_def=path.join(
                     PROJECT_ROOT, 'src/third_party/yolo/config/yolov3.cfg'),
                 weights_path=path.join(
                     PROJECT_ROOT, 'src/third_party/yolo/weights/yolov3.weights'),
                 class_path=path.join(
                     PROJECT_ROOT, 'src/third_party/yolo/data/coco.names')
                 ):
        self.nms_thres = nms_thres
        self.conf_thres = conf_thres
        self.model_def = model_def
        self.weights_path = weights_path
        self.class_path = class_path
        self.device = torch.device(
            'cuda' if torch.cuda.is_available() else 'cpu')
        # Extracts class labels from file
        self.classes = load_classes(self.class_path)
        self.Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() else torch.FloatTensor
        self.model_loaded = False

    def load_model(self, height, width):
        # Set up model
        self.img_size = self.size_scale(height, width)
        self.model = Darknet(
            self.model_def, img_size=self.img_size).to(self.device)
        self.model.load_darknet_weights(self.weights_path)
        self.model.eval()  # Set in evaluation mode
        self.model_loaded = True
        print("Load Yolo model succeeded!")

    def detect(self, np_array, display_result=False):
        if not self.model_loaded:
            self.load_model(np_array.shape[0], np_array.shape[1])

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

            if display_result:
                self.display(np_array, detections)

        return result

    def dummy_detect(self, sleep_time):
        time.sleep(sleep_time)
        print('Sleep Time: %s' % sleep_time)
        result = [
            {'x': 20,
             'y': 20,
             'width': 50,
             'height': 100,
             'score': 0.8,
             'label': 'panda'
             }
        ]
        return result

    def size_scale(self, height, width):
        average = (height + width) / 2
        if average < 416:
            result = average
        else:
            result = 416 + 300 * math.tanh((average - 416) / 500.0)
        return int(result)

    def display(self, np_array, detections):
        # Bounding-box colors
        cmap = plt.get_cmap('tab20b')
        colors = [cmap(i) for i in np.linspace(0, 1, 20)]
        # Create plot
        plt.figure()
        ax = plt.axes()
        ax.imshow(np_array)

        # Draw bounding boxes and labels of detections
        unique_labels = detections[:, -1].cpu().unique()
        n_cls_preds = len(unique_labels)
        bbox_colors = random.sample(colors, n_cls_preds)
        for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
            print('\t+ Label: %s, Conf: %.5f' %
                  (self.classes[int(cls_pred)], cls_conf.item()))

            box_w = x2 - x1
            box_h = y2 - y1

            color = bbox_colors[int(
                np.where(unique_labels == int(cls_pred))[0])]
            # Create a Rectangle patch
            bbox = patches.Rectangle(
                (x1, y1), box_w, box_h, linewidth=2, edgecolor=color, facecolor='none')
            # Add the bbox to the plot
            ax.add_patch(bbox)
            # Add label
            plt.text(
                x1,
                y1,
                s=self.classes[int(cls_pred)],
                color='white',
                verticalalignment='top',
                bbox={'color': color, 'pad': 0},
            )

        # Show generated image with detections
        plt.axis('off')
        plt.gca().xaxis.set_major_locator(NullLocator())
        plt.gca().yaxis.set_major_locator(NullLocator())
        plt.show()
        plt.close()


if __name__ == '__main__':
    yolo = Yolo()
    image_path = path.join(path.dirname(path.abspath(__file__)), 'test.png')
    print(yolo.detect(np.array(Image.open(image_path)), display_result=True))
