from __future__ import division

from yolo_e1.models import *
from yolo_e1.utils.utils import *
from yolo_e1.utils.datasets import *

import os
import sys
import time
import datetime
import argparse
import numpy as np

from PIL import Image

import torch
from torch.utils.data import DataLoader
from torchvision import datasets
from torch.autograd import Variable

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.ticker import NullLocator
from config import HEIGHT, WIDTH


class Yolo:
    def __init__(self, img_size=WIDTH,
                 nms_thres=0.4,
                 conf_thres=0.8,
                 show_result=False,
                 model_def='yolo_e1/config/yolov3.cfg',
                 weights_path='yolo_e1/weights/yolov3.weights',
                 class_path='yolo_e1/data/coco.names'
                 ):
        self.img_size = img_size
        self.nms_thres = nms_thres
        self.conf_thres = conf_thres
        self.show_result = show_result
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

        if self.show_result:
            # Bounding-box colors
            cmap = plt.get_cmap('tab20b')
            colors = [cmap(i) for i in np.linspace(0, 1, 20)]
            # Create plot
            plt.figure()
            ax = plt.axes()
            ax.imshow(np_array)

            # Draw bounding boxes and labels of detections
            if detections is not None:
                # Rescale boxes to original image
                detections = rescale_boxes(
                    detections, self.img_size, np_array.shape[:2])
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
    yolo = Yolo(show_result=True)
    yolo.detect(np.array(Image.open('test.png')))
