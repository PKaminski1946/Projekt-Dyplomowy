import zmq
import pickle
import cv2
import time
import screeninfo

RECORD = False

FPS = 15
delay = int(1/FPS*1000)

IMG_W = 640
IMG_H = 360

context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.connect("tcp://localhost:5555")

cv2.namedWindow("FPV camera", cv2.WINDOW_NORMAL)
cv2.setWindowProperty("FPV camera", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

monitor = screeninfo.get_monitors()[0]

while True:
    img = pickle.loads(socket.recv())
    cv2.imshow('FPV camera', img)

    if RECORD:
        out.write(img)

    cv2.waitKey(delay)

