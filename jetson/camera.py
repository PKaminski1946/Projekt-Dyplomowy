import zmq
import pickle
import pyrealsense2 as rs
import numpy as np
import time

IMG_W = 640
IMG_H = 360
CAMERA_FPS = 15

def main():
    #set up push server
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind("tcp://*:5555")
    
    print("sever up")

    #set up camera
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        dev.hardware_reset()
    
    #not waiting after reset causes an error for some reason
    time.sleep(1)
    
    config = rs.config()
    config.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.rgb8, CAMERA_FPS)
    
    pipeline = rs.pipeline()
    pipeline.start(config)
    
    print("camera up")

    while True:    
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
            
        socket.send(pickle.dumps(color_image))
        
if __name__ == "__main__":
    main()
