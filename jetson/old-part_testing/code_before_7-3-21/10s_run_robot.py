import cv2
import json
import numpy as np
import imagezmq
import socket
import time
from threaded_cam import jetson_csi_camera
import sys
from utils.apriltag_reader import apriltag_reader

def cvtto3channal(frame):
    frame_3chan = np.zeros([frame.shape[0],frame.shape[1],3])
    frame_3chan[:,:,0] = frame
    frame_3chan[:,:,1] = frame
    frame_3chan[:,:,2] = frame
    return frame_3chan

# def region_of_interest(frame, ROI_pts):
#     mask = np.zeros_like(frame)
#     cv2.fillPoly(mask, ROI_pts, (255,255,255))
#     # print(frame.shape)
#     ROIed_frame = cv2.bitwise_and(frame, mask)
#     return ROIed_frame

# def region_of_interest(frame, ROI_pts, topY):
#     frame = frame[topY : frame.shape[0]][:]
#     mask = np.zeros_like(frame)
#     cv2.fillPoly(mask, ROI_pts, (255,255,255))
#     # print(frame.shape)
#     ROIed_frame = cv2.bitwise_and(frame, mask)
#     return ROIed_frame

def region_of_interest(frame, ROI_pts):
    mask = np.zeros_like(frame)
    cv2.fillPoly(mask, ROI_pts, (255,255,255))
    ROIed_frame = cv2.bitwise_and(frame, mask)
    return ROIed_frame

def readlane(white_masked):
    global last_error
    ERROR_DIVISION = white_masked.shape[1] / 200
    line_cropped_frame = region_of_interest(white_masked, lane_ROI)
    # print(line_cropped_frame.shape)
    
    contours,hierarchy = cv2.findContours(line_cropped_frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    # Find the biggest contour (if detected)
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        try:
            cx = int(M['m10']/M['m00'])
            # cy = int(M['m01']/M['m00'])
        except ZeroDivisionError:
            cx = 0
            cy = 0
            print("Zero division")

        errors = int(cx/ERROR_DIVISION) - 100
        
        last_error = errors
        return errors
    else: 
        return 0

def pid_control(errors, base_speed, KP, KD):
    derivative = errors - last_error
    output = KP * errors + KD * derivative
    lmspeed = base_speed + errors
    rmspeed = base_speed - errors
    if lmspeed > MAX_SPEED:
        lmspeed = MAX_SPEED
    elif lmspeed < 0:
        lmspeed = 0

    if rmspeed > MAX_SPEED:
        rmspeed = MAX_SPEED
    elif rmspeed < 0:
        rmspeed = 0

    motor(lmspeed, rmspeed)

def main(run_motor):
    try:
        tagsize = 0
        # frame = cv2.imread("640x480_lane_sample.jpg")
        frame = cap.read()
        lasttime = time.time()
        framecount = 0
        ERROR_DIVISION = frame.shape[1] / 200

        while True:
            # if time.time() - lasttime >= 1:
            #     lasttime = time.time()
            #     print(framecount)
            #     framecount = 0
            # framecount += 1

            frame = cap.read()
            HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
            laneline_masked = cv2.inRange(HSV_frame,lowerY,upperY)
            laneline_masked = cv2.medianBlur(laneline_masked, 7)
            errors = readlane(laneline_masked)
            errors -= 40
            print(errors)
            
            tags = tag.read(frame)
            if tags:
                tagsize = tags[131]
                if tagsize > 15:
                    break

            if run_motor:
                pid_control(errors, BASE_SPEED, KP, KD)

            # cv2.polylines(mask3chan,lane_ROI,True,(0,0,255),2)
            # publish_frame = np.hstack([frame,mask3chan])
            # ret_code, jpg_buffer = cv2.imencode(
            #         ".jpg", publish_frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
            # sender.send_jpg(rpi_name, jpg_buffer)
            
    # except Exception as e:
    except KeyboardInterrupt:
        # print(e)
        if run_motor:
            motor(0, 0)
        print("In exception block probaly exiting properly.")
        cap.stop()
        sys.exit()

def find_mask_pix_area(img):
    return np.sum(img == 255)  

def run_until_red(run_motor):
    try:
        tagsize = 0
        # frame = cv2.imread("640x480_lane_sample.jpg")
        frame = cap.read()
        lasttime = time.time()
        framecount = 0
        ERROR_DIVISION = frame.shape[1] / 200
        BASE_SPEED = 20
        state = 0
        start_time = time.time()
        KP = 0.5
        while True:
            # if time.time() - lasttime >= 1:
            #     lasttime = time.time()
            #     print(framecount)
            #     framecount = 0

            frame = cap.read()
            HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
            red_masked = cv2.inRange(HSV_frame,lowerR,upperR)
            red_masked = cv2.medianBlur(red_masked,7)
            
            white_masked = cv2.inRange(HSV_frame,lowerY,upperY)
            white_masked = cv2.medianBlur(white_masked, 7)

            if (find_mask_pix_area(red_masked) > 8000):
                break

            errors = readlane(white_masked)
            errors -= 40
            
            if time.time() - start_time < 3:
                KP = 0.3
                BASE_SPEED = 40
            else:
                KP = 0.1
                BASE_SPEED = 127
            # print(errors)
            
            if run_motor:
                pid_control(errors, BASE_SPEED, KP, KD)

            # cv2.polylines(mask3chan,lane_ROI,True,(0,0,255),2)
            # publish_frame = np.hstack([frame,mask3chan])
            # ret_code, jpg_buffer = cv2.imencode(
            #         ".jpg", publish_frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
            # sender.send_jpg(rpi_name, jpg_buffer)
            framecount += 1
            
    # except Exception as e:
    except KeyboardInterrupt:
        # print(e)
        if run_motor:
            motor(0,0)
        print("In exception block probaly exiting properly.")
        cap.stop()
        sys.exit()


if __name__ == '__main__':
    KP = 0.2
    KD = 0
    BASE_SPEED = 127
    MAX_SPEED = 127
    last_error = 0

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--motor', action='store_true')
    parser.add_argument('--stream', action='store_true')
    parser.add_argument('--camera', action='store_true')
    args = parser.parse_args()

    if args.motor:
        from motor_control import *
        print("motor run")
    
    CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

    if args.stream:
        IMAGE_PORT = 5555
        sender = imagezmq.ImageSender("tcp://*:{}".format(IMAGE_PORT), REQ_REP=False)
        jpeg_quality = 60
        rpi_name = socket.gethostname()
        print("Input stream opened")

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

    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    BUTTON_PIN = 18
    GPIO.setup(BUTTON_PIN, GPIO.IN)

    while GPIO.input(BUTTON_PIN) == 1:
        pass

    tag = apriltag_reader([131])

    # topY = [lane_ROI[0][0][1], lane_ROI[0][1][1]]
    # highestY = max(topY)

    # lane_ROI[0][0][1] = np.array([0], np.int32)
    # lane_ROI[0][1][1] = np.array([0], np.int32)
    time.sleep(0.5)
    
    main(args.motor)
    motor(50, 50)
    try:
        while True:
            print("going to tag")
            frame = cap.read()
            tags = tag.read(frame)
            # print(tags)
            if tags:
                tagsize = tags[131]
                print(tags)
                if tagsize > 32:
                    break

        motor(0,90)
        time.sleep(0.5)
        motor(0,0)
        run_until_red(args.motor)
        motor(127,127)
        time.sleep(0.5)
        motor(0, 0)
    except KeyboardInterrupt:
        # print(e)
        motor(0, 0)
        cap.stop()
        sys.exit()

    
    print("running")
    # run_until_red(args.motor)
    
    
    motor(0, 0)
    cap.stop()
    sys.exit()