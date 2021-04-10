import apriltag
import cv2
from threaded_cam import jetson_csi_camera
import imagezmq
import sys
import socket as sk

CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

cap = jetson_csi_camera(CAMSET)
detector = apriltag.Detector()

port = 5555
sender = imagezmq.ImageSender("tcp://*:{}".format(port), REQ_REP=False)
print("Input stream opened")
JPEG_QUALITY = 30
sender_name = sk.gethostname()

try:
    while True:
        img = cap.read()

        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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
            print(ptB[0] - ptA[0])
            cv2.line(img, ptA, ptB, (0, 255, 0), 2)
            cv2.line(img, ptB, ptC, (0, 255, 0), 2)
            cv2.line(img, ptC, ptD, (0, 255, 0), 2)
            cv2.line(img, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            tagid = str(r.tag_id)
            cv2.putText(img, tagid, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        ret_code, jpg_buffer = cv2.imencode(
        ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        sender.send_jpg(sender_name, jpg_buffer)

except:
    cap.stop()
    sys.exit()