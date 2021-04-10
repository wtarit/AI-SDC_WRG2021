import zmq
import cv2
import numpy as np
import threading
import time

# Creates a socket instance
context = zmq.Context()
socket = context.socket(zmq.SUB)
# socket.RCVTIMEO = 1000
host = "127.0.0.1"
port = "5001"
# Connects to a bound socket
socket.connect("tcp://{}:{}".format(host, port))

# Subscribes to all topics
socket.subscribe("configs")

configs_data = ""
old_configs_data = ""

# def subscribe_configs():
#     while True:
#         global configs_data
#         socket.recv_string()
#         configs_data = socket.recv_json()

# reciever = threading.Thread(target=subscribe_configs, daemon=True)
# reciever.start()

i = 0
while True:
    socket.recv_string()
    print(socket.recv_json())
    i += 1
    print(i)
# poller = zmq.Poller()
# poller.register(socket, zmq.POLLIN)

# while True:
#     evts = dict(poller.poll(timeout=100))
#     if socket in evts:
#         topic = socket.recv_string()
#         status = socket.recv_json()
#         print(f"Topic: {topic} => {status}")