import cv2
import numpy as np
from utils.threaded_cam import jetson_csi_camera
from utils.stream_image import ZMQ_img_stream
import time

DIM=(320, 240)
K=np.array([[152.66815760452354, 0.0, 165.31994260707836], [0.0, 153.00151818623013, 100.70125534259182], [0.0, 0.0, 1.0]])
D=np.array([[-0.04142400025363713], [0.05019522766966924], [-0.09195816349598072], [0.05138471379317989]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(frame):
    h,w = frame.shape[:2]
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

if __name__ == '__main__':
    stream = ZMQ_img_stream(True)
    CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    cap = jetson_csi_camera(CAMSET)
    try:
        lasttime = time.time()
        framecount = 0
        while True:
            if time.time() - lasttime >= 1:
                lasttime = time.time()
                print(framecount)
                framecount = 0
            framecount += 1
            frame = cap.read()
            frame = undistort(frame)
            stream.send_frame([frame])
    except Exception as e:
        print(e)
    finally:
        cap.stop()
