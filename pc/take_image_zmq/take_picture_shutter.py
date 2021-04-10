import tkinter as tk
import cv2
from PIL import Image, ImageTk
import numpy as np
from ZMQ_receive_class import VideoStreamSubscriber
import os
import zmq

host = "0.0.0.0"
port = "5001"

# Creates a socket instance
context = zmq.Context()
socket = context.socket(zmq.PUB)

# Binds the socket to a predefined port on localhost
socket.bind("tcp://{}:{}".format(host, port))

hostname = os.getenv("ROBOT_IP")  # Use to receive from other computer
# hostname = "192.168.1.128"
vid_port = 5555
receiver = VideoStreamSubscriber(hostname, vid_port)
width, height = 640, 480

root = tk.Tk()

lmain = tk.Label(root)
lmain.pack()

def show_frame():
    msg, frame = receiver.receive()
    frame = cv2.imdecode(np.frombuffer(frame, dtype='uint8'), -1)
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
    img = Image.fromarray(cv2image)
    imgtk = ImageTk.PhotoImage(image=img)
    lmain.imgtk = imgtk
    lmain.configure(image=imgtk)
    lmain.after(10, show_frame)

def testButton():
    # Sends a string message
    socket.send_string("take_photo")
    print('saved')

save_button = tk.Button(root,text = 'save',command = testButton)
save_button.pack()

show_frame()
root.mainloop()
