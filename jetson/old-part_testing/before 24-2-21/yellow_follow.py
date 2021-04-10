import cv2
import json
import numpy as np
import imagezmq
import socket
import time
from threaded_cam import jetson_csi_camera
import sys
import apriltag

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

def readlane(frame):
    global last_error
    ERROR_DIVISION = frame.shape[1] / 200
    HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
    white_masked = cv2.inRange(HSV_frame,lowerY,upperY)
    white_masked = cv2.medianBlur(white_masked,7)
    #mask3chan = cvtto3channal(white_masked)
    # line_cropped_frame = region_of_interest(white_masked, lane_ROI, highestY)
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
        
        # cv2.circle(frame,(cx,cy),3,(0,0,255),5)
        # cv2.drawContours(frame, contours, -1, (0,255,0), 2)
    #print("time_taken",time.time()-prev_time)
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

    motor(lmspeed + 127,rmspeed + 127)


KP = 0.2
KD = 1
BASE_SPEED = 60
MAX_SPEED = 127
last_error = 0

def main(run_motor):
    try:
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

            frame = cap.read()
            img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            results = detector.detect(img_gray)
            # print(results)
            
            for r in results:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = r.corners
                
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                # draw the bounding box of the AprilTag detection
                tagsize = ptB[0] - ptA[0]
                # cv2.line(img, ptA, ptB, (0, 255, 0), 2)
                # cv2.line(img, ptB, ptC, (0, 255, 0), 2)
                # cv2.line(img, ptC, ptD, (0, 255, 0), 2)
                # cv2.line(img, ptD, ptA, (0, 255, 0), 2)
                # draw the center (x, y)-coordinates of the AprilTag
                tagid = str(r.tag_id)
                # cv2.putText(img, tagid, (ptA[0], ptA[1] - 15),
                #     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # frame = cv2.imread("640x480_lane_sample.jpg")
            errors = readlane(frame)
            errors -= 40
            
            
            if tagsize > 35:
                break
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
            motor(127,127)
        print("In exception block probaly exiting properly.")
        cap.stop()
        sys.exit()

if __name__ == '__main__':
    detector = apriltag.Detector()
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--motor', action='store_true')
    parser.add_argument('--stream', action='store_true')
    parser.add_argument('--camera', action='store_true')
    args = parser.parse_args()

    if args.motor:
        from motor_control import *
        print("motor run")
    
    # CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    # CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=120/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    # CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=120/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

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


    except FileNotFoundError as identifier:
        print("no lane config found")
        print("Please configs your robot first.")
        sys.exit()

    if args.camera:
        cap = jetson_csi_camera(CAMSET)

    # topY = [lane_ROI[0][0][1], lane_ROI[0][1][1]]
    # highestY = max(topY)

    # lane_ROI[0][0][1] = np.array([0], np.int32)
    # lane_ROI[0][1][1] = np.array([0], np.int32)
    time.sleep(0.5)
    
    main(args.motor)
    
    motor(127, 127)
    cap.stop()
    sys.exit()


    