import zmq
import fire
import numpy as np
from config import HEIGHT, WIDTH
from yolo_e1_ import Yolo


def server(ipc_file_path):
    # ipc_file_path: e.g. /tmp/image_detection.ipc
    yolo = Yolo()
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("ipc://" + ipc_file_path)
    print("Binded to ipc://" + ipc_file_path)
    while True:
        message = socket.recv()
        np_array = np.frombuffer(message, dtype=np.uint8).reshape(
            (HEIGHT, WIDTH, 3))
        yolo.detect(np_array)
        socket.send(b"Inference successed!")


def main():
    fire.Fire(server)


if __name__ == '__main__':
    main()
