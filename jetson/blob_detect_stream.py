import sys
import socket
import cv2
import imagezmq
import time
import numpy as np
import json
from utils.threaded_cam import jetson_csi_camera
from utils.stream_image import ZMQ_img_stream

CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

cap = jetson_csi_camera(CAMSET)
stream = ZMQ_img_stream(True)

DIM=(320, 240)
K=np.array([[152.66815760452354, 0.0, 165.31994260707836], [0.0, 153.00151818623013, 100.70125534259182], [0.0, 0.0, 1.0]])
D=np.array([[-0.04142400025363713], [0.05019522766966924], [-0.09195816349598072], [0.05138471379317989]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(frame):
    h,w = frame.shape[:2]
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

try:
    with open("lane_config.json") as f:
        data = json.load(f)
    lowerB = data["B_hue_min"],data["B_sat_min"],data["B_val_min"]
    upperB = data["B_hue_max"],data["B_sat_max"],data["B_val_max"]

except FileNotFoundError as identifier:
    print("no lane config found")
    print("Please configs your robot first.")
    sys.exit()

detector = cv2.SimpleBlobDetector()

try:
    framecount = 0
    lasttime = time.time()
    while True:
        frame = cap.read()
        frame = undistort(frame)
        HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
        blue_masked = cv2.inRange(HSV_frame,lowerB,upperB)
        blue_masked = cv2.medianBlur(blue_masked, 7)
        contours, hierarchy = cv2.findContours(blue_masked, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Find the biggest contour (if detected)
        if len(contours) > 0:
            c = max(contours, key = cv2.contourArea)
            print(cv2.contourArea(c))
        
except Exception as e:
    print(e)

finally:
    cap.stop()
    sys.exit()
