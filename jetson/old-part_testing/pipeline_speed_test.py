import cv2
from threaded_cam import jetson_csi_camera
from utils.apriltag_reader import apriltag_reader
from utils.lane_reader import lane

cap = jetson_csi_camera()