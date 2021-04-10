import cv2
import numpy as np

class lane:
    def __init__(self, ROI_pts):
        self.errors = 0
        self.last_errors = 0
        self.ROI_pts = ROI_pts
        self.cx = None
        self.cy = None

    def region_of_interest(self, frame):
        mask = np.zeros_like(frame)
        cv2.fillPoly(mask, self.ROI_pts, (255,255,255))
        ROIed_frame = cv2.bitwise_and(frame, mask)
        return ROIed_frame

    def readlane_maxarea(self, white_masked):
        ERROR_DIVISION = white_masked.shape[1] / 200
        self.line_cropped_frame = self.region_of_interest(white_masked)
        # print(line_cropped_frame.shape)
        
        self.contours, self.hierarchy = cv2.findContours(self.line_cropped_frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Find the biggest contour (if detected)
        if len(self.contours) > 0:
            c = max(self.contours, key = cv2.contourArea)
            M = cv2.moments(c)
            try:
                self.cx = int(M['m10']/M['m00'])
                self.cy = int(M['m01']/M['m00'])
            except ZeroDivisionError:
                self.cx = 0
                self.cy = 0
                print("Zero division")
                return self.last_errors

            errors = int(self.cx / ERROR_DIVISION) - 100
            
            self.last_errors = errors
            return errors
        else: 
            return self.last_errors

    def readlane_maxY_nowarp(self, white_masked):
        ERROR_DIVISION = white_masked.shape[1] / 200
        self.line_cropped_frame = self.region_of_interest(white_masked)
        # print(line_cropped_frame.shape)
        
        self.contours, self.hierarchy = cv2.findContours(self.line_cropped_frame, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Find the biggest contour (if detected)
        centroids = []
        for c in self.contours:
            M = cv2.moments(c)
            try:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centroids.append([cx, cy])
            except ZeroDivisionError:
                pass
                # print("Zero division")

        if len(centroids) > 0:
            centroids.sort(key=lambda x: x[1])
            self.cx = centroids[0][0]
            self.cy = centroids[0][1]
            errors = int(self.cx / ERROR_DIVISION) - 100
        
            self.last_errors = errors
            return errors

        else:
            return self.last_errors

    def readlane_maxY(self, white_masked):
        ERROR_DIVISION = white_masked.shape[1] / 200
        # print(line_cropped_frame.shape)
        
        self.contours, self.hierarchy = cv2.findContours(white_masked, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        # Find the biggest contour (if detected)
        centroids = []
        for c in self.contours:
            M = cv2.moments(c)
            try:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centroids.append([cx, cy])
            except ZeroDivisionError:
                pass
                # print("Zero division")

        if len(centroids) > 0:
            centroids.sort(key=lambda x: x[1])
            self.cx = centroids[0][0]
            self.cy = centroids[0][1]
            errors = int(self.cx / ERROR_DIVISION) - 100
        
            self.last_errors = errors
            return errors

        else:
            return self.last_errors

    def get_lane_visualisation(self, rgb_frame):
        cv2.polylines(rgb_frame, self.ROI_pts, True, (0, 0, 255), 2)
        if self.cx and self.cy is not None:
            cv2.circle(rgb_frame,(self.cx, self.cy), 3, (0,0,255), 5)
            cv2.drawContours(rgb_frame, self.contours, -1, (0,255,0), 2)
        return rgb_frame

if __name__ == "__main__":
    import json
    import sys

    frame = cv2.imread("lane_test.jpg")
    frame = cv2.resize(frame, (320, 240))
    try:
        with open("configurator_using_zmq/lane_config.json") as f:
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

    lane_det = lane(lane_ROI)

    lowerY = np.array([22, 0, 32])
    upperY = np.array([39, 183, 223])

    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    frame_mask = cv2.inRange(frame_HSV, lowerY, upperY)
    error = lane_det.readlane_maxarea(frame_mask)
    frame  = lane_det.get_lane_visualisation(frame)
    cv2.imshow("mask", frame_mask)
    cv2.imshow("frame", frame)
    
    cv2.waitKey(0)