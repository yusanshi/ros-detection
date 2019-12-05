import zmq
import time

context= zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("ipc:///tmp/reverse_string.ipc") # TODO

while True:
    message = socket.recv()
    print("Received request: %s" % message)
    time.sleep(0.05)
    reply = message[::-1]
    socket.send(reply)
    print("Sent reply: %s" % reply)