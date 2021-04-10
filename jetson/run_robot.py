import cv2
import json
import numpy as np
import imagezmq
import socket
import time
from utils.threaded_cam import jetson_csi_camera
import sys
from utils.apriltag_reader import apriltag_reader
from utils.lane_reader import lane

DIM=(320, 240)
K=np.array([[152.66815760452354, 0.0, 165.31994260707836], [0.0, 153.00151818623013, 100.70125534259182], [0.0, 0.0, 1.0]])
D=np.array([[-0.04142400025363713], [0.05019522766966924], [-0.09195816349598072], [0.05138471379317989]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(frame):
    h,w = frame.shape[:2]
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

def cvtto3channal(frame):
    frame_3chan = np.zeros([frame.shape[0],frame.shape[1],3])
    frame_3chan[:,:,0] = frame
    frame_3chan[:,:,1] = frame
    frame_3chan[:,:,2] = frame
    return frame_3chan

def run_to_tag():
    KP = 0.8
    KD = 0
    BASE_SPEED = 127
    MAX_SPEED = 127
    tagsize = 0
    # frame = cv2.imread("640x480_lane_sample.jpg")
    frame = cap.read()
    lasttime = time.time()
    framecount = 0
    ERROR_DIVISION = frame.shape[1] / 200

    while True:
        if time.time() - lasttime >= 1:
            lasttime = time.time()
            print(framecount)
            framecount = 0
        framecount += 1
        
        # start_time = time.time()
        frame = cap.read()
        frame = undistort(frame)
        # print((time.time() - start_time) * 1000)
        # frame = frame[120:240][:]
        HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
        laneline_masked = cv2.inRange(HSV_frame, lowerY, upperY)
        laneline_masked = cv2.medianBlur(laneline_masked, 7)
        warp = cv2.warpPerspective(laneline_masked, M, (320, 240))
        errors = lane_det.readlane_maxY(warp)
        errors -= 30

        
        tags = tag.read(frame)
        if tags:
            tagsize = tags[131]
            if tagsize > 10:
                break

        motor.pid_control(errors, BASE_SPEED, KP, KD)

def find_mask_pix_area(img):
    return np.sum(img == 255)  

def follow_lane():
    # frame = cv2.imread("640x480_lane_sample.jpg")
    frame = cap.read()
    lasttime = time.time()
    framecount = 0
    ERROR_DIVISION = frame.shape[1] / 200
    # BASE_SPEED = 127
    BASE_SPEED = 100
    KP = 1
    KD = 0
    LONGITUDUNAL_KP = 0
    run_speed = 0
    start_t = time.time()
    while time.time() - start_t < 0.9:
        if time.time() - lasttime >= 1:
            lasttime = time.time()
            print(framecount)
            framecount = 0
        framecount += 1

        frame = cap.read()
        frame = undistort(frame)
        HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
        
        laneline_masked = cv2.inRange(HSV_frame,lowerY,upperY)
        laneline_masked = cv2.medianBlur(laneline_masked, 7)

        warp = cv2.warpPerspective(laneline_masked, M, (320, 240))
        errors = lane_det.readlane_maxY(warp)

        errors -= 20
        
        run_speed = BASE_SPEED - (abs(errors) * LONGITUDUNAL_KP)

        motor.pid_control(errors, run_speed, KP, KD)
        print(run_speed, errors)

def run_until_red():
    # frame = cv2.imread("640x480_lane_sample.jpg")
    frame = cap.read()
    lasttime = time.time()
    framecount = 0
    ERROR_DIVISION = frame.shape[1] / 200
    BASE_SPEED = 127
    KP = 0.2
    KD = 0
    while True:
        # if time.time() - lasttime >= 1:
        #     lasttime = time.time()
        #     print(framecount)
        #     framecount = 0
        # framecount += 1

        frame = cap.read()
        frame = undistort(frame)
        HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
        red_masked = cv2.inRange(HSV_frame,lowerR,upperR)
        red_masked = cv2.medianBlur(red_masked,7)
        
        laneline_masked = cv2.inRange(HSV_frame,lowerY,upperY)
        laneline_masked = cv2.medianBlur(laneline_masked, 7)

        if (find_mask_pix_area(red_masked) > 15000):
            break
        
        warp = cv2.warpPerspective(laneline_masked, M, (320, 240))
        errors = lane_det.readlane_maxY(warp)

        errors -= 30

        motor.pid_control(errors, BASE_SPEED, KP, KD)
        print(errors)

def run_to_can():
    frame = cap.read()
    lasttime = time.time()
    framecount = 0
    BASE_SPEED = 127
    KP = 2
    KD = 0
    LONGITUDUNAL_KP = 0
    run_speed = 0
    start_t = time.time()
    while time.time() - start_t < 1:
        if time.time() - lasttime >= 1:
            lasttime = time.time()
            print(framecount)
            framecount = 0
        framecount += 1

        frame = cap.read()
        frame = undistort(frame)
        HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
        
        laneline_masked = cv2.inRange(HSV_frame,lowerY,upperY)
        laneline_masked = cv2.medianBlur(laneline_masked, 7)

        warp = cv2.warpPerspective(laneline_masked, M, (320, 240))
        errors = lane_det.readlane_maxY(warp)

        # errors -= 20
        
        run_speed = BASE_SPEED - (abs(errors) * LONGITUDUNAL_KP)

        motor.pid_control(errors, run_speed, KP, KD)
        print(run_speed, errors)

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--motor', action='store_true')
    parser.add_argument('--stream', action='store_true')
    parser.add_argument('--camera', action='store_true')
    parser.add_argument('--print_error', action='store_true')
    args = parser.parse_args()

    from utils.motor_control import Motor
    motor = Motor(args.motor, args.print_error)
    
    CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
        
    try:
        with open("lane_config.json") as f:
            data = json.load(f)
        lowerY = data["W_hue_min"],data["W_sat_min"],data["W_val_min"]
        upperY = data["W_hue_max"],data["W_sat_max"],data["W_val_max"]
        lane_ROI = np.array([[
            (data["X1_lane"],data["Y1_lane"]),
            (data["X2_lane"],data["Y2_lane"]),
            (data["X3_lane"],data["Y3_lane"]),
            (data["X4_lane"],data["Y4_lane"])
        ]],np.int32)
        lowerR = data["R_hue_min"],data["R_sat_min"],data["R_val_min"]
        upperR = data["R_hue_max"],data["R_sat_max"],data["R_val_max"]

    except FileNotFoundError as identifier:
        print("no lane config found")
        print("Please configs your robot first.")
        sys.exit()

    
    if args.camera:
        cap = jetson_csi_camera(CAMSET)
        time.sleep(0.5)

    lane_det = lane(lane_ROI)

    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    BUTTON_PIN = 18
    GPIO.setup(BUTTON_PIN, GPIO.IN)

    tag = apriltag_reader([131])

    # topY = [lane_ROI[0][0][1], lane_ROI[0][1][1]]
    # highestY = max(topY)

    # lane_ROI[0][0][1] = np.array([0], np.int32)
    # lane_ROI[0][1][1] = np.array([0], np.int32)
    rect = np.array([[
            (data["X1_lane"],data["Y1_lane"]),
            (data["X2_lane"],data["Y2_lane"]),
            (data["X3_lane"],data["Y3_lane"]),
            (data["X4_lane"],data["Y4_lane"])
        ]], dtype = "float32")

    maxWidth = 320
    maxHeight = 240

    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    M = cv2.getPerspectiveTransform(rect, dst)

    try:
        while GPIO.input(BUTTON_PIN) == 1:
            pass
        motor.motor(127, 124)
        time.sleep(0.2)
        run_to_tag()
        motor.motor(100, 100)
        while True:
            # print("going to tag")
            frame = cap.read()
            tags = tag.read(frame)
            # print(tags)
            if tags:
                tagsize = tags[131]
                # print(tags)
                if tagsize > 21:
                    break

        motor.motor(10,127)
        time.sleep(0.45)
        motor.motor(0,0)
        follow_lane()
        print("start to go to red")
        # motor.motor(0, 0)
        # time.sleep(1)
        run_until_red()
        motor.motor(127, 127)
        time.sleep(0.05)
        motor.motor(0,0)

    except Exception as e:
        print(e)
    finally:
        cap.stop()
        GPIO.cleanup()
        sys.exit()