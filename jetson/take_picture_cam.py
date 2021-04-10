import sys
import socket as sk
import cv2
from threading import Thread
import threading
import zmq
from utils.threaded_cam import jetson_csi_camera
from utils.blocking_stream_image import ZMQ_img_stream
import os

CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
cap = jetson_csi_camera(CAMSET)
take_photo = False

# host = "192.168.1.197"
host = "10.42.0.1"
port = "5001"

# Creates a socket instance
context = zmq.Context()
socket = context.socket(zmq.SUB)

# Connects to a bound socket
socket.connect("tcp://{}:{}".format(host, port))

# Subscribes to all topics
socket.subscribe("")

def subscribe_configs():
    global take_photo
    while True:
        if socket.recv_string():
            take_photo = True

reciever = threading.Thread(target=subscribe_configs, daemon=True)
reciever.start()

stream = ZMQ_img_stream(True)
try:
    foldername = "captured_picture"
    isdir = os.path.isdir(foldername)
    if not isdir:
        #create directory
        os.makedirs(foldername)
    
    picture_num = 0
    while True:
        frame = cap.read()
        if take_photo:
            filename = 'captured_picture/'+str(picture_num)+'.jpg'
            cv2.imwrite(filename,frame)
            picture_num += 1
            print('saved as', filename)
            take_photo = False
        stream.send_frame([frame])

except Exception as e:
    print(e)

finally:
    cap.stop()
    sys.exit()

