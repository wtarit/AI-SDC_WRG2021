import tkinter as tk
import numpy as np
import json
import zmq
import socket as sk

host = "0.0.0.0"

port = "5001"

# Creates a socket instance
context = zmq.Context()
socket = context.socket(zmq.PUB)

# Binds the socket to a predefined port on localhost
socket.bind("tcp://{}:{}".format(host, port))

#used to configs ROI trackbar
ROI_width = 320
ROI_height = 240

root = tk.Tk()

tosend = np.zeros([2,3],np.int16)
def send_configs_data(x):
    configs = {
        "W_hue_min":WHue_min.get(),
        "W_sat_min":WSat_min.get(),
        "W_val_min":WVal_min.get(),
        "W_hue_max":WHue_max.get(),
        "W_sat_max":WSat_max.get(),
        "W_val_max":WVal_max.get(),
        "X1_lane":X1_lane.get(),
        "Y1_lane":Y1_lane.get(),
        "X2_lane":X2_lane.get(),
        "Y2_lane":Y2_lane.get(),
        "X3_lane":X3_lane.get(),
        "Y3_lane":Y3_lane.get(),
        "X4_lane":X4_lane.get(),
        "Y4_lane":Y4_lane.get(),
        "R_hue_min":RHue_min.get(),
        "R_sat_min":RSat_min.get(),
        "R_val_min":RVal_min.get(),
        "R_hue_max":RHue_max.get(),
        "R_sat_max":RSat_max.get(),
        "R_val_max":RVal_max.get(),

        "B_hue_min":BHue_min.get(),
        "B_sat_min":BSat_min.get(),
        "B_val_min":BVal_min.get(),
        "B_hue_max":BHue_max.get(),
        "B_sat_max":BSat_max.get(),
        "B_val_max":BVal_max.get()
    }
    socket.send_string("configs",flags=zmq.SNDMORE)
    socket.send_json(configs)


#####configs for lane coloe
WHue_min_label =  tk.Label(root,text = "WHue_min").grid(row = 0,column = 0)
WHue_min = tk.Scale(root,from_=0,to = 180,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
WHue_min.grid(row=0,column=1)

WSat_min_label =  tk.Label(root,text = "WSat_min").grid(row = 1,column = 0)
WSat_min = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
WSat_min.grid(row=1,column=1)

WVal_min_label =  tk.Label(root,text = "WVal_min").grid(row = 2,column = 0)
WVal_min = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
WVal_min.grid(row=2,column=1)

WHue_max_label =  tk.Label(root,text = "WHue_max").grid(row = 3,column = 0)
WHue_max = tk.Scale(root,from_=0,to = 180,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
WHue_max.grid(row=3,column=1)

WSat_max_label =  tk.Label(root,text = "WSat_max").grid(row = 4,column = 0)
WSat_max = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
WSat_max.grid(row=4,column=1)

WVal_max_label =  tk.Label(root,text = "WVal_max").grid(row = 5,column = 0)
WVal_max = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
WVal_max.grid(row=5,column=1)

########line ROI
X1_lane_label =  tk.Label(root,text = "X1_lane").grid(row = 0,column = 2)
X1_lane = tk.Scale(root,from_=0,to = ROI_width ,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
X1_lane.grid(row=0,column=3)

Y1_lane_label =  tk.Label(root,text = "Y1_lane").grid(row = 1,column = 2)
Y1_lane = tk.Scale(root,from_=0,to = ROI_height,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
Y1_lane.grid(row=1,column=3)

X2_lane_label =  tk.Label(root,text = "X2_lane").grid(row = 2,column = 2)
X2_lane = tk.Scale(root,from_=0,to = ROI_width,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
X2_lane.grid(row=2,column=3)

Y2_lane_label =  tk.Label(root,text = "Y2_lane").grid(row = 3,column = 2)
Y2_lane = tk.Scale(root,from_=0,to = ROI_height,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
Y2_lane.grid(row=3,column=3)

X3_lane_label =  tk.Label(root,text = "X3_lane").grid(row = 4,column = 2)
X3_lane = tk.Scale(root,from_=0,to = ROI_width,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
X3_lane.grid(row=4,column=3)

Y3_lane_label =  tk.Label(root,text = "Y3_lane").grid(row = 5,column = 2)
Y3_lane = tk.Scale(root,from_=0,to = ROI_height,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
Y3_lane.grid(row=5,column=3)

X4_lane_label =  tk.Label(root,text = "X4_lane").grid(row = 6,column = 2)
X4_lane = tk.Scale(root,from_=0,to = ROI_width,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
X4_lane.grid(row=6,column=3)

Y4_lane_label =  tk.Label(root,text = "Y4_lane").grid(row = 7,column = 2)
Y4_lane = tk.Scale(root,from_=0,to = ROI_height,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
Y4_lane.grid(row=7,column=3)

#######ROI for red line
BHue_min_label =  tk.Label(root,text = "BHue_min").grid(row = 0,column = 4)
BHue_min = tk.Scale(root,from_=0,to = 180,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
BHue_min.grid(row=0,column=5)

BSat_min_label =  tk.Label(root,text = "BSat_min").grid(row = 1,column = 4)
BSat_min = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
BSat_min.grid(row=1,column=5)

BVal_min_label =  tk.Label(root,text = "BVal_min").grid(row = 2,column = 4)
BVal_min = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
BVal_min.grid(row=2,column=5)

BHue_max_label =  tk.Label(root,text = "BHue_max").grid(row = 3,column = 4)
BHue_max = tk.Scale(root,from_=0,to = 180,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
BHue_max.grid(row=3,column=5)

BSat_max_label =  tk.Label(root,text = "BSat_max").grid(row = 4,column = 4)
BSat_max = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
BSat_max.grid(row=4,column=5)

BVal_max_label =  tk.Label(root,text = "BVal_max").grid(row = 5,column = 4)
BVal_max = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
BVal_max.grid(row=5,column=5)

#######configs for red line color
RHue_min_label =  tk.Label(root,text = "RHue_min").grid(row = 0,column = 6)
RHue_min = tk.Scale(root,from_=0,to = 180,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
RHue_min.grid(row=0,column=7)

RSat_min_label =  tk.Label(root,text = "WSat_min").grid(row = 1,column = 6)
RSat_min = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
RSat_min.grid(row=1,column=7)

RVal_min_label =  tk.Label(root,text = "WVal_min").grid(row = 2,column = 6)
RVal_min = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
RVal_min.grid(row=2,column=7)

RHue_max_label =  tk.Label(root,text = "WHue_max").grid(row = 3,column = 6)
RHue_max = tk.Scale(root,from_=0,to = 180,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
RHue_max.grid(row=3,column=7)

RSat_max_label =  tk.Label(root,text = "WSat_max").grid(row = 4,column = 6)
RSat_max = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
RSat_max.grid(row=4,column=7)

RVal_max_label =  tk.Label(root,text = "WVal_max").grid(row = 5,column = 6)
RVal_max = tk.Scale(root,from_=0,to = 255,orient=tk.HORIZONTAL,length = 255,command = send_configs_data)
RVal_max.grid(row=5,column=7)


try:
    with open("lane_config.json") as f:
        data = json.load(f)
    X1_lane.set(data["X1_lane"])
    Y1_lane.set(data["Y1_lane"])
    X2_lane.set(data["X2_lane"])
    Y2_lane.set(data["Y2_lane"])
    X3_lane.set(data["X3_lane"])
    Y3_lane.set(data["Y3_lane"])
    X4_lane.set(data["X4_lane"])
    Y4_lane.set(data["Y4_lane"])

    WHue_min.set(data["W_hue_min"])
    WSat_min.set(data["W_sat_min"])
    WVal_min.set(data["W_val_min"])
    WHue_max.set(data["W_hue_max"])
    WSat_max.set(data["W_sat_max"])
    WVal_max.set(data["W_val_max"])

    RHue_min.set(data["R_hue_min"])
    RSat_min.set(data["R_sat_min"])
    RVal_min.set(data["R_val_min"])
    RHue_max.set(data["R_hue_max"])
    RSat_max.set(data["R_sat_max"])
    RVal_max.set(data["R_val_max"])

    BHue_min.set(data["B_hue_min"])
    BSat_min.set(data["B_sat_min"])
    BVal_min.set(data["B_val_min"])
    BHue_max.set(data["B_hue_max"])
    BSat_max.set(data["B_sat_max"])
    BVal_max.set(data["B_val_max"])

except FileNotFoundError as identifier:
    print("no lane config found")

######save all configs
def save_lane_configs_button():
    configs = {
        "W_hue_min":WHue_min.get(),
        "W_sat_min":WSat_min.get(),
        "W_val_min":WVal_min.get(),
        "W_hue_max":WHue_max.get(),
        "W_sat_max":WSat_max.get(),
        "W_val_max":WVal_max.get(),
        "X1_lane":X1_lane.get(),
        "Y1_lane":Y1_lane.get(),
        "X2_lane":X2_lane.get(),
        "Y2_lane":Y2_lane.get(),
        "X3_lane":X3_lane.get(),
        "Y3_lane":Y3_lane.get(),
        "X4_lane":X4_lane.get(),
        "Y4_lane":Y4_lane.get(),
        "R_hue_min":RHue_min.get(),
        "R_sat_min":RSat_min.get(),
        "R_val_min":RVal_min.get(),
        "R_hue_max":RHue_max.get(),
        "R_sat_max":RSat_max.get(),
        "R_val_max":RVal_max.get(),

        "B_hue_min":BHue_min.get(),
        "B_sat_min":BSat_min.get(),
        "B_val_min":BVal_min.get(),
        "B_hue_max":BHue_max.get(),
        "B_sat_max":BSat_max.get(),
        "B_val_max":BVal_max.get()
    }
    record = json.dumps(configs,indent=4)
    with open("lane_config.json", "w") as outfile: 
        outfile.write(record)

    print('line configs saved')

lane_save_button = tk.Button(root,text = 'save_lane_configs',command = save_lane_configs_button)
lane_save_button.grid(row = 14,column = 1)

root.mainloop()