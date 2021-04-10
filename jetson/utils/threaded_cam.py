import cv2
import threading

class still_camera:
    def __init__(self, filename):
        self.img = cv2.imread(filename)
    def read(self):
        return self.img.copy()
    def stop(self):
        pass

class ThreadedCamera:
    def __init__(self, resolution=(640,480),camera_id = 0,fps = 30):
        self._stopped = False
        self._cap = cv2.VideoCapture(camera_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution[0])
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        #read the first frame
        self._ret , self._frame = self._cap.read()
        threading.Thread(target=self.run, args=()).start()

    def run(self):
        while not self._stopped:
            self._ret , self._frame = self._cap.read()

    def read(self):
        return self._frame

    def stop(self):
        self._stopped = True

class jetson_csi_camera:
    def __init__ (self,gstreamer_pipeline_string) :
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            
        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)
            return
        # Grab the first frame to start the video capturing
        self.grabbed, self.frame = self.video_capture.read()

        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running=True
            self.read_thread = threading.Thread(target=self.updateCamera, daemon=True)
            self.read_thread.start()
        
    def stop(self):
        self.running=False
        self.video_capture.release()

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            _, self.frame = self.video_capture.read()
        

    def read(self):
        # with self.read_lock:
        #     frame = self.frame
        #    grabbed=self.grabbed
        #return grabbed, frame
        return self.frame.copy()