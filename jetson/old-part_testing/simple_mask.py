import cv2
import numpy as np
from threaded_cam import jetson_csi_camera
CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=3280, height=2464, framerate=21/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=640, height=480, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
cap = jetson_csi_camera(CAMSET)
# trackbar callback fucntion to update HSV value


def callback(x):
    global H_low, H_high, S_low, S_high, V_low, V_high
    # assign trackbar position value to H,S,V High and low variable
    H_low = cv2.getTrackbarPos('low H', 'controls')
    H_high = cv2.getTrackbarPos('high H', 'controls')
    S_low = cv2.getTrackbarPos('low S', 'controls')
    S_high = cv2.getTrackbarPos('high S', 'controls')
    V_low = cv2.getTrackbarPos('low V', 'controls')
    V_high = cv2.getTrackbarPos('high V', 'controls')


# create a seperate window named 'controls' for trackbar
cv2.namedWindow('controls')


#global variable
H_low = 0
H_high = 179
S_low = 0
S_high = 255
V_low = 0
V_high = 255

# create trackbars for high,low H,S,V
cv2.createTrackbar('low H', 'controls', 0, 179, callback)
cv2.createTrackbar('high H', 'controls', 179, 179, callback)

cv2.createTrackbar('low S', 'controls', 0, 255, callback)
cv2.createTrackbar('high S', 'controls', 255, 255, callback)

cv2.createTrackbar('low V', 'controls', 0, 255, callback)
cv2.createTrackbar('high V', 'controls', 255, 255, callback)


while(1):
    # read source image
    img = cap.read()
    #img = cv2.GaussianBlur(img, (3,3), 0)
    #img = cv2.medianBlur(img,5)
    # convert sourece image to HSC color mode
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    #hsv = cv2.medianBlur(hsv,5)
    
    hsv_low = np.array([H_low, S_low, V_low], np.uint8)
    hsv_high = np.array([H_high, S_high, V_high], np.uint8)

    # making mask for hsv range
    #mask = cv2.inRange(hsv, hsv_low, hsv_high)
    mask = cv2.inRange(hsv, np.array([154,115,74]), np.array([180,255,255]))
    mask = cv2.medianBlur(mask,7)
    #print(mask)
    # masking HSV value selected color becomes black
    #res = cv2.bitwise_and(img, img, mask=mask)

    # show image
    cv2.imshow('mask', mask)
    cv2.imshow('res', img)

    # waitfor the user to press escape and break the while loop
    k = cv2.waitKey(1)
    if k == 27:
        break

cap.stop()
# destroys all window
cv2.destroyAllWindows()
