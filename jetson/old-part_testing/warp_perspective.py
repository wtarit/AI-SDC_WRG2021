import cv2
from utils.threaded_cam import jetson_csi_camera
from utils.stream_image import ZMQ_img_stream
from utils.lane_reader import lane
import json
import numpy as np

CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
cap = jetson_csi_camera(CAMSET)
stream = ZMQ_img_stream(True)

def cvtto3channal(frame):
    frame_3chan = np.zeros([frame.shape[0],frame.shape[1],3])
    frame_3chan[:,:,0] = frame
    frame_3chan[:,:,1] = frame
    frame_3chan[:,:,2] = frame
    return frame_3chan

DIM=(320, 240)
K=np.array([[152.66815760452354, 0.0, 165.31994260707836], [0.0, 153.00151818623013, 100.70125534259182], [0.0, 0.0, 1.0]])
D=np.array([[-0.04142400025363713], [0.05019522766966924], [-0.09195816349598072], [0.05138471379317989]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def undistort(frame):
    h,w = frame.shape[:2]
    undistorted_img = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img

try:
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
    
    lane_det = lane(lane_ROI)
    maxWidth = 320
    maxHeight = 240
    rect = np.array([[
            (data["X1_lane"],data["Y1_lane"]),
            (data["X2_lane"],data["Y2_lane"]),
            (data["X3_lane"],data["Y3_lane"]),
            (data["X4_lane"],data["Y4_lane"])
        ]], dtype = "float32")

    dst = np.array([
        [0, 0],
        [maxWidth - 1, 0],
        [maxWidth - 1, maxHeight - 1],
        [0, maxHeight - 1]], dtype = "float32")
    M = cv2.getPerspectiveTransform(rect, dst)
    while True:
        frame = cap.read()
        frame = undistort(frame)
        HSV_frame = cv2.cvtColor(frame,cv2.COLOR_BGR2HLS)
        laneline_masked = cv2.inRange(HSV_frame, lowerY, upperY)
        laneline_masked = cv2.medianBlur(laneline_masked, 7)
        warp = cv2.warpPerspective(laneline_masked, M, (320, 240))
        errors = lane_det.readlane_maxY(warp)
        print(errors - 40)
        
        masked_3_chan = cvtto3channal(warp)
        masked_3_chan = lane_det.get_lane_visualisation(masked_3_chan)
        stream.send_frame([frame, masked_3_chan])
        
except Exception as e:
    print(e)
finally:
    cap.stop()