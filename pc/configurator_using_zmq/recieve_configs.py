import zmq
import cv2
import numpy as np
import threading
import time
import imagezmq
import socket as sk

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

port = 5555
sender = imagezmq.ImageSender("tcp://*:{}".format(port), REQ_REP=False)
print("Input stream opened")
jpeg_quality = 95
sender_name = sk.gethostname()


# Creates a socket instance
context = zmq.Context()
socket = context.socket(zmq.SUB)
host = "127.0.0.1"
port = "5001"
# Connects to a bound socket
socket.connect("tcp://{}:{}".format(host, port))

# Subscribes to all topics
socket.subscribe("configs")

configs_data = ""
old_configs_data = ""

def subscribe_configs():
    while True:
        global configs_data
        socket.recv_string()
        configs_data = socket.recv_json()

reciever = threading.Thread(target=subscribe_configs, daemon=True)
reciever.start()

while True:
    _ ,frame = cap.read()
    ret_code, jpg_buffer = cv2.imencode(
    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
    sender.send_jpg(sender_name, jpg_buffer)

    if configs_data != old_configs_data:
        print(configs_data)
        old_configs_data = configs_data

# poller = zmq.Poller()
# poller.register(socket, zmq.POLLIN)

# while True:
#     evts = dict(poller.poll(timeout=100))
#     if socket in evts:
#         topic = socket.recv_string()
#         status = socket.recv_json()
#         print(f"Topic: {topic} => {status}")