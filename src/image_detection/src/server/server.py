import os
import sys
import zmq
import fire
import json
import numpy as np
from yolo import Yolo


def server():
    if 'ROS_NAMESPACE' not in os.environ:
        sys.exit('ROS_NAMESPACE not set.')

    ros_namespace = os.environ['ROS_NAMESPACE']
    ros_namespace = '/' + ros_namespace.replace('/', '')
    ipc_file_path = '/tmp' + ros_namespace + '.ipc'
    yolo = Yolo()
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind('ipc://' + ipc_file_path)
    print('Binded to ipc://' + ipc_file_path)
    while True:
        multipart = socket.recv_multipart()
        img_size = json.loads(multipart[0])
        np_array = np.frombuffer(multipart[1], dtype=np.uint8).reshape(
            (img_size['height'], img_size['width'], 3))
        result = yolo.detect(np_array)
        # result = yolo.dummy_detect(0.033)
        socket.send_json(result)


if __name__ == '__main__':
    fire.Fire(server)
