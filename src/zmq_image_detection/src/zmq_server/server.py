import zmq
import fire
import cv2
import numpy as np
from PIL import Image
from config import HEIGHT, WIDTH
import yolo_e1_


def server(ipc_file_path):
    # ipc_file_path: e.g. /tmp/image_detection.ipc
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("ipc://" + ipc_file_path)
    print("Binded to ipc://" + ipc_file_path)

    while True:
        message = socket.recv()
        np_array = np.frombuffer(message, dtype=np.uint8).reshape(
            (HEIGHT, WIDTH, 3))
        result = yolo_e1_.detect(np_array)
        socket.send(b"test")


def main():
    fire.Fire(server)


if __name__ == '__main__':
    main()
