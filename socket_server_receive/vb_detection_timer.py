import cv2
import pickle
from lbp import LocalBinaryPatterns
import time 

import socket

HOST = "192.168.1.2"  # The server's hostname or IP address
PORT = 10000  # The port used by the server

vb_send = [0, 0.0]

model_weights = './trained_NuSVC.sav'
model = pickle.load(open(model_weights, 'rb'))

resize = True
size = 512
display = True
ret_err = 0 

desc = LocalBinaryPatterns(24, 8)

# cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
# cap = cv2.VideoCapture('GH040143.MP4')
cap = cv2.VideoCapture('output.mp4')

if not cap.isOpened():
    raise Exception("The camera was not initialized correctly")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

while True: 
    ret,img=cap.read()

    sum_dt = 0.0
    frame_count = 0
    dt_array = []

    if ret:
        # tick = time.perf_counter()
        tick = time.time()
        ret_err = 0
        gray = cv2.cvtColor(cv2.resize(img, (size,size), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY)
        hist = desc.describe(gray)
        prediction = model.predict(hist.reshape(1, -1))
        lr_probs = model.predict_proba(hist.reshape(1, -1))
        # tock = time.perf_counter()
        tock = time.time()
        dt = tock - tick
        
        vb_send = [prediction[0], lr_probs[0][prediction[0]]]

        # print(type(prediction[0]),type(lr_probs[0][prediction[0]]))

        # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # s.connect((HOST, PORT))
        print("sending", vb_send, "to server")
        s.sendall(str(vb_send).encode())
                  
        # print(prediction[0],lr_probs[0][prediction[0]])
        
        print("Time per frame (secs) =", dt)
        sum_dt += dt
        frame_count += 1

        dt_array.append(dt)

    ret_err +=1
    if ret_err > 600:
        mean_dt = sum_dt / frame_count
      
        s = 0.0

        for i in range(frame_count):
            s += (dt_array[i] - mean_dt)**2

        std = (s / frame_count)**0.5

        print("Mean time per frame (secs) =", mean_dt)
        print("Overall standard deviation of time per frame (secs)=", std)

        raise Exception("600 empty frames were recieved")
