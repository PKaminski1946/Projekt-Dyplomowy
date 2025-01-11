import cv2
import numpy as np
import pyyolo

import time
import os
import serial
import collections
import Jetson.GPIO as GPIO
import threading
import pyrealsense2 as rs
import pickle
import zmq

#camera image size
IMG_H = 360	
IMG_W = 640

#camera FPS
CAMERA_FPS = 15 

IMG_H_C = IMG_H / 2
IMG_W_C = IMG_W / 2
IMG_S = (IMG_H * IMG_W) ** 0.5

#target object to image ratio
TARG_SR = 0.012 #0.01
TARG_SR = np.sqrt(TARG_SR)
S_QUEUE_LEN = 3

MAX_ROLL = 1
MIN_ROLL = -1
MAX_PITCH = 1
MIN_PITCH = -1
MAX_THROTTLE = 0.436
MIN_THROTTLE = 0.260

#autonomous mode only
AUTO_ONLY = True

#SBUS constants
SBUS_BAUD = 100000
SBUS_HEADER = 0x0f 
SBUS_FOOTER = 0x00

#mode switching time
SWITCHING_TIME_S = 3

#serial port initialization
serial_port = serial.Serial(port = "/dev/ttyTHS1", bytesize=serial.EIGHTBITS, baudrate=100000, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO)

#yolo initialization
detector = pyyolo.YOLO('/home/patryk/darknet/cfg/yolov7-tiny-drone.cfg', '/home/patryk/darknet/yolov7-tiny-train_best.weights', '/home/patryk/darknet/data/drone.data')

#position variables
det_x = IMG_H_C
det_y = IMG_W_C
det_s_queue = collections.deque(S_QUEUE_LEN*[TARG_SR*IMG_S], S_QUEUE_LEN)

#current control variables
curr_control_vals = np.zeros(4) #roll, pitch, throttle, yaw

#target control variables
target_control_vals = np.zeros(4) #roll, pitch, throttle, yaw

#analog channels
analog_channels = np.zeros(16, dtype=np.uint16)

#initialize camera
ctx = rs.context()
devices = ctx.query_devices()
for dev in devices:
    dev.hardware_reset()

#not waiting after reset causes an error
time.sleep(1)

config = rs.config()
config.enable_stream(rs.stream.color, IMG_W, IMG_H, rs.format.rgb8, CAMERA_FPS)

pipeline = rs.pipeline()
pipeline.start(config)

#PID
class Controller():
    def __init__(self, sp ,kp, ki, kd, min_control_val, max_control_val):
        self.sp = sp
        self.kp = kp
        self.Ti = ki
        self.Td = kd

        self.min_con = min_control_val
        self.max_con = max_control_val

        self.err = 0
        self.err_int = 0
        self.prev_err = 0
        self.err_diff = 0
        self.time_prev = None

    def update(self, x):
        if self.time_prev is None:
            self.time_prev = time.time()

        curr_time = time.time()
        self.err = (self.sp - x) / self.sp

        self.err_int += self.err * 1/self.Ti * (curr_time - self.time_prev)
        # self.err_int = np.clip(self.err_int, self.min_con, self.max_con)
        self.err_int = np.clip(self.err_int, -3, 3)
        self.err_diff = self.Td*(self.err - self.prev_err)/(curr_time - self.time_prev)
        y = self.kp * (self.err + self.err_int + self.err_diff)
        y = np.clip(y, -1, 1)

        self.prev_err = self.err
        self.time_prev = curr_time
        return np.interp(y, [-1, 1], [self.min_con, self.max_con])

#PID controllers
roll_controller = Controller(IMG_W_C, 0.3, float('inf'), 1.73, MIN_ROLL, MAX_ROLL) 
pitch_controller = Controller(IMG_H_C, 0.3, float('inf'), 1.73, MIN_PITCH, MAX_PITCH) 
throttle_controller = Controller(TARG_SR*IMG_S, 0.55, float('inf'), 4.8, MIN_THROTTLE, MAX_THROTTLE) 


#switching variables
switch_time = 0
is_manual = True
switch_start_vals = np.zeros(4)

def detect_target():
    global target_control_vals

    #connect to camera process
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind("tcp://*:5555")
    print("socket up")
    while True:
        #retrieve frame 
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())
        
        disp_img = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

        #object detection
        dets = detector.detect(color_image, rgb=False)

        target_detected = False

        for i, det in enumerate(dets):

            if det.name == "car" or det.name == "van":
                target_detected = True
                xmin, ymin, xmax, ymax = det.to_xyxy()

                det_x = int((xmin + xmax) / 2)
                det_y = int((ymin + ymax) / 2)
                det_s = ((xmax - xmin) * (ymax - ymin)) ** 0.5

                if np.abs(det_x - det_x) > 1:
                    det_x = det_x

                if np.abs(det_y - det_y) > 1:
                    det_y = det_y

                disp_img = cv2.rectangle(disp_img, (xmin, ymin), (xmax, ymax), (0, 0, 255))
                break

        socket.send(pickle.dumps(disp_img))

        if target_detected:
            target_control_vals[0] = -roll_controller.update(det_x)
            target_control_vals[2] = throttle_controller.update(2*TARG_SR*IMG_S - det_s)
            target_control_vals[1] = pitch_controller.update(det_y)
            target_control_vals[3] = 0
        
        else:
            target_control_vals = [0, 0, (MIN_THROTTLE+MAX_THROTTLE)/2, 0]


def create_sbus_packet(analog_channels, digital_channels = np.zeros(2, dtype = "uint16")):
    sbus_packet = np.zeros(25, dtype = "uint8")

    sbus_packet[0] = 0x0f

    sbus_packet[1] = ((analog_channels[0] & 0x07ff))
    sbus_packet[2] = ((analog_channels[0] & 0x07ff) >> 8 | (analog_channels[1] & 0x07ff) << 3)
    sbus_packet[3] = ((analog_channels[1] & 0x07FF) >> 5 | (analog_channels[2] & 0x07FF) << 6)
    sbus_packet[4] = ((analog_channels[2] & 0x07FF) >> 2)
    sbus_packet[5] = ((analog_channels[2] & 0x07FF) >> 10 | (analog_channels[3] & 0x07FF) << 1)
    sbus_packet[6] = ((analog_channels[3] & 0x07FF) >> 7 | (analog_channels[4] & 0x07FF) << 4)
    sbus_packet[7] = ((analog_channels[4] & 0x07FF) >> 4 | (analog_channels[5] & 0x07FF) << 7)
    sbus_packet[8] = ((analog_channels[5] & 0x07FF) >> 1)
    sbus_packet[9] = ((analog_channels[5] & 0x07FF) >> 9 | (analog_channels[6] & 0x07FF) << 2)
    sbus_packet[10] = ((analog_channels[6] & 0x07FF) >> 6 | (analog_channels[7] & 0x07FF) << 5)
    sbus_packet[11] = ((analog_channels[7] & 0x07FF) >> 3)
    sbus_packet[12] = ((analog_channels[8] & 0x07FF))
    sbus_packet[13] = ((analog_channels[8] & 0x07FF) >> 8 | (analog_channels[9] & 0x07FF) << 3)
    sbus_packet[14] = ((analog_channels[9] & 0x07FF) >> 5 | (analog_channels[10] & 0x07FF) << 6)
    sbus_packet[15] = ((analog_channels[10] & 0x07FF) >> 2)
    sbus_packet[16] = ((analog_channels[10] & 0x07FF) >> 10 | (analog_channels[11] & 0x07FF) << 1)
    sbus_packet[17] = ((analog_channels[11] & 0x07FF) >> 7 | (analog_channels[12] & 0x07FF) << 4)
    sbus_packet[18] = ((analog_channels[12] & 0x07FF) >> 4 | (analog_channels[13] & 0x07FF) << 7)
    sbus_packet[19] = ((analog_channels[13] & 0x07FF) >> 1)
    sbus_packet[20] = ((analog_channels[13] & 0x07FF) >> 9 | (analog_channels[14] & 0x07FF) << 2)
    sbus_packet[21] = ((analog_channels[14] & 0x07FF) >> 6 | (analog_channels[15] & 0x07FF) << 5)
    sbus_packet[22] = ((analog_channels[15] & 0x07FF) >>3 )

    sbus_packet[23] = ((digital_channels[1] << 1) | (digital_channels[0]))

    sbus_packet[24] = 0x00

    return sbus_packet

def unpack_sbus_packet(received_data):
    analog_channels = np.zeros(16, dtype = "uint16")
    digital_channels = np.zeros(2, dtype = "uint16")

    analog_channels[0]  = ((received_data[0]    |received_data[1] <<8)                     & 0x07FF)
    analog_channels[1]  = ((received_data[1]>>3 |received_data[2] <<5)                     & 0x07FF)
    analog_channels[2]  = ((received_data[2]>>6 |received_data[3] <<2 |received_data[4]<<10)    & 0x07FF)
    analog_channels[3]  = ((received_data[4]>>1 |received_data[5] <<7)                     & 0x07FF)
    analog_channels[4]  = ((received_data[5]>>4 |received_data[6] <<4)                     & 0x07FF)
    analog_channels[5]  = ((received_data[6]>>7 |received_data[7] <<1 |received_data[8]<<9)     & 0x07FF)
    analog_channels[6]  = ((received_data[8]>>2 |received_data[9] <<6)                     & 0x07FF)
    analog_channels[7]  = ((received_data[9]>>5 |received_data[10]<<3)                     & 0x07FF)
    analog_channels[8]  = ((received_data[11]   |received_data[12]<<8)                     & 0x07FF)
    analog_channels[9]  = ((received_data[12]>>3|received_data[13]<<5)                     & 0x07FF)
    analog_channels[10] = ((received_data[13]>>6|received_data[14]<<2 |received_data[15]<<10)   & 0x07FF)
    analog_channels[11] = ((received_data[15]>>1|received_data[16]<<7)                     & 0x07FF)
    analog_channels[12] = ((received_data[16]>>4|received_data[17]<<4)                     & 0x07FF)
    analog_channels[13] = ((received_data[17]>>7|received_data[18]<<1 |received_data[19]<<9)    & 0x07FF)
    analog_channels[14] = ((received_data[19]>>2|received_data[20]<<6)                     & 0x07FF)
    analog_channels[15] = ((received_data[20]>>5|received_data[21]<<3)                     & 0x07FF)

    digital_channels[0] = received_data[22] & 0x01
    digital_channels[1] = received_data[22] & 0x02

    return analog_channels

def receive_sbus_packet():
    received_data = np.zeros(23, dtype = "uint16")
    current_byte = b'/x00'
    previous_byte = b'/x00'
    byte_idx = 0

    while True:
        current_byte = serial_port.read(1)
        
        if (byte_idx == 0):
            if int.from_bytes(current_byte, "big") == SBUS_HEADER and int.from_bytes(previous_byte, "big") == SBUS_FOOTER:
                byte_idx = 1

        else:
            if (byte_idx == 24):
                return received_data

            else:
                received_data[byte_idx-1] = int.from_bytes(current_byte, "big")
                byte_idx += 1

        previous_byte = current_byte

def get_manual_control_data():
    control_channels = np.zeros(5, dtype = "uint16")
    received_data = receive_sbus_packet()

    control_channels[0]  = ((received_data[0]    |received_data[1] <<8)                     & 0x07FF)
    control_channels[1]  = ((received_data[1]>>3 |received_data[2] <<5)                     & 0x07FF)
    control_channels[2]  = ((received_data[2]>>6 |received_data[3] <<2 |received_data[4]<<10)    & 0x07FF)
    control_channels[3]  = ((received_data[4]>>1 |received_data[5] <<7)                     & 0x07FF)
    control_channels[4]  = ((received_data[5]>>4 |received_data[6] <<4)                     & 0x07FF)

    return control_channels

def control_update():
    global curr_control_vals
    global target_control_vals
    global analog_channels
    global switch_time
    global is_manual

    while True:
        if AUTO_ONLY:
            is_manual = False
        
        else:
            serial_port.flushInput()
            manual_control_channels = unpack_sbus_packet(receive_sbus_packet())
            
            is_manual = manual_control_channels[4] < 1500
        
        scaled_target_control_vals = np.copy(target_control_vals)

        #convert control values to [0, 1]
        scaled_target_control_vals[0] += 0.5
        scaled_target_control_vals[1] += 0.5
        scaled_target_control_vals[3] += 0.5

        #map floats to int range recognized by the flight controller
        scaled_target_control_vals = np.round(np.interp(scaled_target_control_vals, [0, 1], [176, 1811]))
        
        elapsed_time = 0

        if is_manual:
            curr_control_vals[2] = manual_control_channels[2]
            curr_control_vals[0] = manual_control_channels[0]
            curr_control_vals[1] = manual_control_channels[1]
            curr_control_vals[3] = manual_control_channels[3]
            
            switch_time = None

        else:
            if switch_time is None:
                switch_time = time.time()
                switch_start_vals = curr_control_vals

            elapsed_time = time.time() - switch_time
            
            if elapsed_time < SWITCHING_TIME_S:
                curr_control_vals = (scaled_target_control_vals - switch_start_vals) / SWITCHING_TIME_S * elapsed_time + switch_start_vals

            else:
                curr_control_vals = np.copy(scaled_target_control_vals)
        
        curr_control_vals = np.round(curr_control_vals)
        analog_channels[:4] = np.copy(curr_control_vals)  
        analog_channels = analog_channels.astype(np.uint16)
        
        print(f"{curr_control_vals}")

        #transmit control values over UART
        serial_port.write(bytes(create_sbus_packet(analog_channels)))

        #write data to log
        #update_log()

        time.sleep(0.05)

def create_log():
    log_list = os.listdir("/home/patryk/Documents/flight_logs")
    max_id = 0
    for log_name in log_list:
        log_num = int(log_name[4:-4])

        if log_num > max_id:
            max_id = log_num

    log = open(f"/home/patryk/Documents/flight_logs/log_{max_id+1}.txt", "w") 
    log_start_time = time.time()

def update_log(log):
    log.write(f"{time.time() - log_start_time} {det_x} {det_y} {det_s} {curr_control_vals[0]} {curr_control_vals[1]} {curr_control_vals[2]} {curr_control_vals[3]} {int(is_manual)}\n")  


def main():

    #create log file
    #create_log()

    obj_det_thread = threading.Thread(target=detect_target)
    control_update_thread = threading.Thread(target=control_update)
    obj_det_thread.start()
    control_update_thread.start()

if __name__ == "__main__":
    main()
