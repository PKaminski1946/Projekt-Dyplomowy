import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import numpy as np
import pyyolo

import time
import os
import serial
import collections
import Jetson.GPIO as GPIO
import threading
import zmq
import pickle

from std_msgs.msg import String, Bool, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from px4_msgs.msg import ManualControlSetpoint, VehicleLocalPosition


#camera image size
IMG_H = 360	
IMG_W = 640

IMG_H_C = IMG_H / 2
IMG_W_C = IMG_W / 2
IMG_S = (IMG_H * IMG_W) ** 0.5

X_QUEUE_LEN = 6
Y_QUEUE_LEN = 6

#target object to image ratio
TARG_SR = 0.011 #0.01
TARG_SR = np.sqrt(TARG_SR)
S_QUEUE_LEN = 3

MAX_ROLL = 1
MIN_ROLL = -1
MAX_PITCH = 1
MIN_PITCH = -1
MAX_THROTTLE = 0.58
MIN_THROTTLE = 0.1

#autonomous mode only
AUTO_ONLY = False #True

#zmq socket for display script
context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.bind("tcp://*:5555")

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
        self.err_int = np.clip(self.err_int, -0.2, 0.2)
        self.err_diff = self.Td*(self.err - self.prev_err)/(curr_time - self.time_prev)
        y = self.kp * (self.err + self.err_int + self.err_diff)
        y = np.clip(y, -1, 1)

        self.prev_err = self.err
        self.time_prev = curr_time

        return np.interp(y, [-1, 1], [self.min_con, self.max_con])

class Controller_Node(Node):
    def __init__(self, detection_callback_group, timer_callback_group, global_pos_callback_group):
        super().__init__('controller')  

        qos_prof = rclpy.qos.QoSProfile(history = rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth = 1)
        sickly_victorian_child_qos = rclpy.qos.QoSProfile(reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT, depth=1)

        self.subscription = self.create_subscription(Image, '/camera', self.img_callback, qos_prof, callback_group=detection_callback_group) # raw camera img
        self.subscription
        
        self.global_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.global_pos_callback, sickly_victorian_child_qos, callback_group=global_pos_callback_group)
        self.global_pos_sub

        self.call_control_update = self.create_timer(0.05, self._control_update, callback_group=timer_callback_group)
        
        self.control_publisher = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', 10) # movement control

        #publisher for camera image post detection
        self.post_det_img_publisher = self.create_publisher(Image, '/camera_det', 10)

        self.bridge = CvBridge()

        #uart constants
        self.SBUS_BAUD = 100000
        self.SBUS_HEADER = 0x0f 
        self.SBUS_FOOTER = 0x00

        self.SWITCHING_TIME_S = 3

        self.serial_port = serial.Serial(port = "/dev/ttyTHS1", bytesize=serial.EIGHTBITS, baudrate=100000, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO)

        self.detector = pyyolo.YOLO('/home/patryk/darknet/cfg/yolov7-tiny-drone.cfg', '/home/patryk/darknet/yolov7-tiny-train_best.weights', '/home/patryk/darknet/data/drone.data')
       
        #position variables
        self.det_x = IMG_H_C
        self.det_y = IMG_W_C
        self.det_x_queue = collections.deque([IMG_H_C], X_QUEUE_LEN)
        self.det_y_queue = collections.deque([IMG_W_C], Y_QUEUE_LEN)
 
        self.det_s_queue = collections.deque(S_QUEUE_LEN*[TARG_SR*IMG_S], S_QUEUE_LEN)
        self.det_s = self.det_s_queue[0]

        #current control variables
        self.curr_control_vals = np.zeros(4) #roll, pitch, throttle, yaw

        #target control variables
        self.target_control_vals = np.zeros(4) #roll, pitch, throttle, yaw


        self.roll_controller = Controller(IMG_W_C, 0.30, float('inf'), 1.73, MIN_ROLL, MAX_ROLL)
        self.pitch_controller = Controller(IMG_H_C, 0.30, float('inf'), 1.73, MIN_PITCH, MAX_PITCH) 
        self.throttle_controller = Controller(TARG_SR*IMG_S, 0.55, float('inf'), 4.80, MIN_THROTTLE, MAX_THROTTLE)

        #control vals
        self.control_msg = ManualControlSetpoint()
        self.control_msg.data_source = 2
        self.control_msg.valid = True
        self.control_msg.sticks_moving = True

        #switch vars
        self.switch_time = 0
        self.is_manual = True
        self.switch_start_vals = np.zeros(4)

        #global pos - ONLY FOR LOGGING
        self.global_x = 0
        self.global_y = 0
        self.global_z = 0

        #create log file
        self.create_log()

    def global_pos_callback(self, data):      
        self.global_x = data.x
        self.global_y = data.y
        self.global_z = data.z
    
    def img_callback(self, data):
        self.current_frame = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        # self.get_logger().info(f"Video frame received")

        #object detection
        dets = self.detector.detect(self.current_frame, rgb=False)
        
        target_detected = False

        for i, det in enumerate(dets):

            if det.name == "car" or det.name == "van":
                target_detected = True
                xmin, ymin, xmax, ymax = det.to_xyxy()

                det_x = int((xmin + xmax) / 2)
                det_y = int((ymin + ymax) / 2)
                self.det_s = ((xmax - xmin) * (ymax - ymin)) ** 0.5
                self.det_s_queue.append(self.det_s)
                self.det_x_queue.append(self.det_x)
                self.det_y_queue.append(self.det_y)

                if np.abs(self.det_x - det_x) > 1:
                    self.det_x = det_x

                if np.abs(self.det_y - det_y) > 1:
                    self.det_y = det_y

                self.current_frame = cv2.rectangle(self.current_frame, (xmin, ymin), (xmax, ymax), (0, 0, 255))
                break
        #send to display script
        socket.send(pickle.dumps(self.current_frame))
        
        self.target_control_vals[0] = -self.roll_controller.update(np.sum(self.det_x_queue)/X_QUEUE_LEN)
        self.target_control_vals[2] = self.throttle_controller.update(2*TARG_SR*IMG_S - np.sum(self.det_s_queue)/S_QUEUE_LEN)
        self.target_control_vals[1] = self.pitch_controller.update(np.sum(self.det_y_queue)/Y_QUEUE_LEN)
        
    def create_sbus_packet(self, analog_channels, digital_channels = np.zeros(2, dtype = "uint16")):
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

    def unpack_sbus_packet(self, received_data):
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

    def receive_sbus_packet(self):
        received_data = np.zeros(23, dtype = "uint16")
        current_byte = b'/x00'
        previous_byte = b'/x00'
        byte_idx = 0

        while True:
            current_byte = self.serial_port.read(1)
            
            if (byte_idx == 0):
                if int.from_bytes(current_byte, "big") == self.SBUS_HEADER and int.from_bytes(previous_byte, "big") == self.SBUS_FOOTER:
                    byte_idx = 1

            else:
                if (byte_idx == 24):
                    return received_data

                else:
                    received_data[byte_idx-1] = int.from_bytes(current_byte, "big")
                    byte_idx += 1

            previous_byte = current_byte

    def get_manual_control_data(self):
        control_channels = np.zeros(5, dtype = "uint16")
        received_data = self.receive_sbus_packet()

        control_channels[0]  = ((received_data[0]    |received_data[1] <<8)                     & 0x07FF)
        control_channels[1]  = ((received_data[1]>>3 |received_data[2] <<5)                     & 0x07FF)
        control_channels[2]  = ((received_data[2]>>6 |received_data[3] <<2 |received_data[4]<<10)    & 0x07FF)
        control_channels[3]  = ((received_data[4]>>1 |received_data[5] <<7)                     & 0x07FF)
        control_channels[4]  = ((received_data[5]>>4 |received_data[6] <<4)                     & 0x07FF)

        return control_channels

    def _control_update(self):
        if AUTO_ONLY:
            self.is_manual = False
        
        else:
            self.serial_port.flushInput()
            manual_control_channels = self.unpack_sbus_packet(self.receive_sbus_packet())
            #self.get_logger().info(f"{manual_control_channels}")
            self.is_manual = manual_control_channels[4] < 1500

            manual_control_channels = ((manual_control_channels - 172) - 819.5) / 409.75 / 2 #convert to -1, 1
            manual_control_channels[2] = (manual_control_channels[2] + 1) / 2
            manual_control_channels[np.abs(manual_control_channels) < 0.03] = 0 #clip noise

        elapsed_time = 0

        if self.is_manual:
            self.curr_control_vals[2] = manual_control_channels[2]
            self.curr_control_vals[0] = manual_control_channels[0]
            self.curr_control_vals[1] = manual_control_channels[1]
            self.curr_control_vals[3] = manual_control_channels[3]
            
            self.switch_time = None

        else:
            if self.switch_time is None:
                self.switch_time = time.time()
                self.switch_start_vals = self.curr_control_vals

            elapsed_time = time.time() - self.switch_time
            
            if elapsed_time < self.SWITCHING_TIME_S:
                self.curr_control_vals = (self.target_control_vals - self.switch_start_vals) / self.SWITCHING_TIME_S * elapsed_time + self.switch_start_vals

            else:
                self.curr_control_vals = np.copy(self.target_control_vals)

        self.control_msg.roll = float(self.curr_control_vals[0]) #roll
        self.control_msg.throttle = float(self.curr_control_vals[2]) #throttle
        self.control_msg.pitch = float(self.curr_control_vals[1]) #pitch
        self.control_msg.yaw = float(self.curr_control_vals[3]) # yaw

        self.get_logger().info(f"{self.curr_control_vals}, {2*TARG_SR*IMG_S - self.det_s, self.throttle_controller.sp}")
        self.control_publisher.publish(self.control_msg)
        
        self.update_log()
 
    def create_log(self):
        log_list = os.listdir("/home/patryk/Documents/flight_logs")
        max_id = 0
        for log_name in log_list:
            log_num = int(log_name[4:-4])

            if log_num > max_id:
                max_id = log_num

        self.log = open(f"/home/patryk/Documents/flight_logs/log_{max_id+1}.txt", "w") 
        self.log_start_time = time.time()

    def update_log(self):
        self.log.write(f"{time.time() - self.log_start_time} {self.det_x} {self.det_y} {self.det_s} {self.curr_control_vals[0]} {self.curr_control_vals[1]} {self.curr_control_vals[2]} {self.curr_control_vals[3]} {int(self.is_manual)} {self.global_x} {self.global_y} {self.global_z} {self.target_control_vals[0]} {self.target_control_vals[1]} {self.target_control_vals[2]} {self.target_control_vals[3]}\n")  


def main():
    rclpy.init()

    img_callback = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    timer_callback = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    global_pos_callback = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    
    controller = Controller_Node(img_callback, timer_callback, global_pos_callback)
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)

    executor.spin()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
