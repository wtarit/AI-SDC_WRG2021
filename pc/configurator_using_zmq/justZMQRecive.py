import sys
import socket
import cv2
from imutils.video import VideoStream
import imagezmq
import threading
import numpy as np
from time import sleep
import os

# Helper class implementing an IO deamon thread
class VideoStreamSubscriber:
    def __init__(self, hostname, port):
        self.hostname = hostname
        self.port = port
        self._stop = False
        self._data_ready = threading.Event()
        self._thread = threading.Thread(target=self._run, args=())
        self._thread.daemon = True
        self._thread.start()

    def receive(self, timeout=15.0):
        flag = self._data_ready.wait(timeout=timeout)
        if not flag:
            raise TimeoutError(
                "Timeout while reading from subscriber tcp://{}:{}".format(self.hostname, self.port))
        self._data_ready.clear()
        return self._data

    def _run(self):
        receiver = imagezmq.ImageHub("tcp://{}:{}".format(self.hostname, self.port), REQ_REP=False)
        while not self._stop:
            self._data = receiver.recv_jpg()
            self._data_ready.set()
        # Close socket here, not implemented in ImageHub :(
        # zmq_socket.close()

    def close(self):
        self._stop = True

#hostname = "10.42.0.240"
# hostname = "10.42.0.174"
# hostname = "192.168.1.128"
hostname = os.getenv("ROBOT_IP")

port = 5555
receiver = VideoStreamSubscriber(hostname, port)

try:
    while True:
        msg, frame = receiver.receive(timeout = 60.0)
        image = cv2.imdecode(np.frombuffer(frame, dtype='uint8'), -1)
        cv2.imshow("Pub Sub Receive", image)
        cv2.waitKey(1)
except (KeyboardInterrupt, SystemExit):
    print('Exit due to keyboard interrupt')
finally:
    receiver.close()
    sys.exit()
