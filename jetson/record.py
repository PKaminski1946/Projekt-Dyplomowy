import zmq 
import pickle 
import cv2 
import time 
import screeninfo 
 
RECORD = True 
 
FPS = 15 
delay = int(1/FPS*1000) 
 
IMG_W = 640
IMG_H = 360

context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.connect("tcp://localhost:5555")
monitor = screeninfo.get_monitors()[0]

if RECORD:
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output.mp4', fourcc, FPS, (IMG_W, IMG_H))

while True:
    img = pickle.loads(socket.recv())
    if RECORD:
        out.write(img)

    cv2.waitKey(delay)
