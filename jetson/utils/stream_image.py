import cv2
import numpy as np
import time
import socket
import imagezmq

class ZMQ_img_stream:
    def __init__(self, stream, img_port = 5555, stream_fps = 20, jpeg_quality = 50):
        self.stream = stream
        self.stream_fps = stream_fps
        self.jpeg_quality = jpeg_quality
        self.looptime = 1/stream_fps
        if self.stream:
            self.sender = imagezmq.ImageSender("tcp://*:{}".format(img_port), REQ_REP=False)
            self.sender_name = socket.gethostname()
            print("Input stream opened")

    def send_frame(self, img):
        if self.stream:
            start_time = time.time()
            if type(img) != list:
                raise TypeError("Input to stream must be a list")
            publish_frame = np.hstack(img)
            ret_code, jpg_buffer = cv2.imencode(
                    ".jpg", publish_frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            self.sender.send_jpg(self.sender_name, jpg_buffer)
            time.sleep(self.looptime - (time.time() - start_time))

if __name__ == "__main__":
    from threaded_cam import jetson_csi_camera
    CAMSET = "nvarguscamerasrc sensor-id=0 tnr-strength=1 tnr-mode=2 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1, format=NV12 ! nvvidconv flip-method=2 ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
    cap = jetson_csi_camera(CAMSET)
    stream = ZMQ_img_stream(True)
    try:
        while True:
            frame = cap.read()
            stream.send_frame([frame, frame])
            
    except Exception as e:
        print(e)
    finally:
        cap.stop()
