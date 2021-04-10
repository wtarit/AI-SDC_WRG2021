import zmq
import cv2
import numpy as np
import threading
import time
import imagezmq
import socket as sk
import os
from utils.threaded_cam import jetson_csi_camera
import sys
import json

#CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
# CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
# CAMSET='nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=3264, height=2464, framerate=21/1,format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
# CAMSET = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

cap = jetson_csi_camera(CAMSET)

port = 5555
sender = imagezmq.ImageSender("tcp://*:{}".format(port), REQ_REP=False)
print("Input stream opened")
JPEG_QUALITY = 30
sender_name = sk.gethostname()

# Creates a socket instance
context = zmq.Context()
socket = context.socket(zmq.SUB)
# host = "192.168.1.197"
host = "10.42.0.1"
port = "5001"
# Connects to a bound socket
socket.connect("tcp://{}:{}".format(host, port))

# Subscribes to all topics
socket.subscribe("configs")

try:
    with open("lane_config.json") as f:
        configs_data = json.load(f)
    lowerY = configs_data["W_hue_min"],configs_data["W_sat_min"],configs_data["W_val_min"]
    upperY = configs_data["W_hue_max"],configs_data["W_sat_max"],configs_data["W_val_max"]

    lowerR = configs_data["R_hue_min"],configs_data["R_sat_min"],configs_data["R_val_min"]
    upperR = configs_data["R_hue_max"],configs_data["R_sat_max"],configs_data["R_val_max"]

    lowerB = configs_data["B_hue_min"],configs_data["B_sat_min"],configs_data["B_val_min"]
    upperB = configs_data["B_hue_max"],configs_data["B_sat_max"],configs_data["B_val_max"]
    
    lane_ROI = np.array([[
        (configs_data["X1_lane"],configs_data["Y1_lane"]),
        (configs_data["X2_lane"],configs_data["Y2_lane"]),
        (configs_data["X3_lane"],configs_data["Y3_lane"]),
        (configs_data["X4_lane"],configs_data["Y4_lane"])
    ]],np.int32)

except FileNotFoundError:
    lowerY = np.zeros([3],np.uint8)
    upperY = np.zeros([3],np.uint8)
    lowerR = np.zeros([3],np.uint8)
    upperR = np.zeros([3],np.uint8)
    lowerB = np.zeros([3],np.uint8)
    upperB = np.zeros([3],np.uint8)
    lane_ROI = np.array([[
        (0,0),
        (0,0),
        (0,0),
        (0,0)
    ]],np.int32)
    print("no lane config found")

def subscribe_configs():
    while True:
        global configs_data
        global lowerY
        global upperY
        global lowerR
        global upperR
        global upperB
        global lowerB
        global lane_ROI
        
        socket.recv_string()
        configs_data = socket.recv_json()

        lowerY = np.array([configs_data["W_hue_min"],configs_data["W_sat_min"],configs_data["W_val_min"]])
        upperY = np.array([configs_data["W_hue_max"],configs_data["W_sat_max"],configs_data["W_val_max"]])

        lowerB = np.array([configs_data["B_hue_min"],configs_data["B_sat_min"],configs_data["B_val_min"]])
        upperB = np.array([configs_data["B_hue_max"],configs_data["B_sat_max"],configs_data["B_val_max"]])
        
        lowerR = np.array([configs_data["R_hue_min"],configs_data["R_sat_min"],configs_data["R_val_min"]])
        upperR = np.array([configs_data["R_hue_max"],configs_data["R_sat_max"],configs_data["R_val_max"]])

        lane_ROI = np.array([[
            (configs_data["X1_lane"],configs_data["Y1_lane"]),
            (configs_data["X2_lane"],configs_data["Y2_lane"]),
            (configs_data["X3_lane"],configs_data["Y3_lane"]),
            (configs_data["X4_lane"],configs_data["Y4_lane"])
        ]],np.int32)

reciever = threading.Thread(target=subscribe_configs, daemon=True)
reciever.start()

DIM=(320, 240)
K=np.array([[152.66815760452354, 0.0, 165.31994260707836], [0.0, 153.00151818623013, 100.70125534259182], [0.0, 0.0, 1.0]])
D=np.array([[-0.04142400025363713], [0.05019522766966924], [-0.09195816349598072], [0.05138471379317989]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(frame):
    h,w = frame.shape[:2]
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

try:
    while True:
        frame = cap.read()
        frame = undistort(frame)
        # frame = cv2.medianBlur(frame, 5)
        HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
        # HSV_frame = cv2.medianBlur(HSV_frame, 5)
        yellow_masked = cv2.inRange(HSV_frame,lowerY,upperY)
        yellow_masked = cv2.medianBlur(yellow_masked,7)

        red_masked = cv2.inRange(HSV_frame,lowerR,upperR)
        red_masked = cv2.medianBlur(red_masked, 5)

        blue_masked = cv2.inRange(HSV_frame,lowerB,upperB)
        blue_masked = cv2.medianBlur(blue_masked, 5)

        mask3chan = np.zeros_like(frame)
        mask3chan[:,:,0] = yellow_masked
        mask3chan[:,:,1] = yellow_masked
        # mask3chan[:,:,2] = red_masked

        # mask3chan[:,:,0] = blue_masked
        # mask3chan[:,:,1] = blue_masked
        # mask3chan[:,:,2] = blue_masked
        
        frame = cv2.polylines(frame,lane_ROI,True,(255,0,0),2)
        mask3chan = cv2.polylines(mask3chan,lane_ROI,True,(0,221,255),2)
        publish_frame = np.hstack([frame,mask3chan])
        # publish_frame = np.hstack([frame, HSV_frame])

        ret_code, jpg_buffer = cv2.imencode(
        ".jpg", publish_frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        sender.send_jpg(sender_name, jpg_buffer)
        

except (KeyboardInterrupt, SystemExit):
    record = json.dumps(configs_data, indent=4)
    with open("lane_config.json", "w") as outfile: 
        outfile.write(record)
    cap.stop()
    sys.exit()
    print('Exit due to keyboard interrupt')
