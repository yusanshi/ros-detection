import os
import zmq
import fire
import numpy as np
from config import HEIGHT, WIDTH
from yolo_e1_ import Yolo


def server():
    if 'ROS_NAMESPACE' not in os.environ:
        print('ROS_NAMESPACE not set.')
    else:
        ros_namespace = os.environ['ROS_NAMESPACE']
        if not ros_namespace.startswith('/'):
            ros_namespace = '/' + ros_namespace
        ipc_file_path = '/tmp' + ros_namespace + '.ipc'
        yolo = Yolo()
        context = zmq.Context()
        socket = context.socket(zmq.REP)
        socket.bind('ipc://' + ipc_file_path)
        print('Binded to ipc://' + ipc_file_path)
        while True:
            message = socket.recv()
            np_array = np.frombuffer(message, dtype=np.uint8).reshape(
                (HEIGHT, WIDTH, 3))
            result = yolo.detect(np_array)
            socket.send_json(result)


if __name__ == '__main__':
    fire.Fire(server)
