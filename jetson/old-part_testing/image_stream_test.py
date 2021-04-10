import sys
import socket
import cv2
import imagezmq

camSet = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=0 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

cap = cv2.VideoCapture(camSet)

port = 5555
sender = imagezmq.ImageSender("tcp://*:{}".format(port), REQ_REP=False)
print("Input stream opened")
jpeg_quality = 95
rpi_name = socket.gethostname()

try:
    while True:
        _ ,frame = cap.read()
        ret_code, jpg_buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
        sender.send_jpg(rpi_name, jpg_buffer)
except (KeyboardInterrupt, SystemExit):
    print('Exit due to keyboard interrupt')
    cap.release()
    sys.exit()
