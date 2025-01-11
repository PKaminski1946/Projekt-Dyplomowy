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

from std_msgs.msg import String, Bool, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from px4_msgs.msg import ManualControlSetpoint


#camera image size
IMG_H = 360	
IMG_W = 640

IMG_H_C = IMG_H / 2
IMG_W_C = IMG_W / 2
IMG_S = (IMG_H * IMG_W)**0.3

#target object to image ratio
TARG_SR = 0.35 #0.07
S_QUEUE_LEN = 1

#max control vals, implemet this in the controller class as a parameter
MAX_ROLL = 1
MIN_ROLL = -1
MAX_PITCH = 1
MIN_PITCH = -1
MAX_THROTTLE = 0.416
MIN_THROTTLE = 0.280

#autonomous mode only
AUTO_ONLY = False

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
        #change this to include min and max vals
        self.err = (self.sp - x) / self.sp #standardize err to [-1, 1]

        self.err_int += self.err * 1/self.Ti * (curr_time - self.time_prev)
        # self.err_int = np.clip(self.err_int, self.min_con, self.max_con)
        self.err_int = np.clip(self.err_int, -3, 3)
        self.err_diff = self.Td*(self.err - self.prev_err)/(curr_time - self.time_prev)
        y = self.kp * (self.err + self.err_int + self.err_diff)
        y = np.clip(y, -1, 1)

        self.prev_err = self.err
        self.time_prev = curr_time

        # if y > 0:
        #     return y * self.max_con

        # elif y < 0:
        #     return y * self.min_con

        return np.interp(y, [-1, 1], [self.min_con, self.max_con])

class Controller_Node(Node):
    def __init__(self, detection_callback_group, timer_callback_group):
        super().__init__('controller')  

        qos_prof = rclpy.qos.QoSProfile(history = rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST, depth = 1)

        self.subscription = self.create_subscription(Image, '/camera', self.img_callback, qos_prof, callback_group=detection_callback_group) # raw camera img
        self.subscription

        self.call_control_update = self.create_timer(0.1, self._control_update, callback_group=timer_callback_group)
        
        self.control_publisher = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', 10) # movement control
        self.control_mode_publisher = self.create_publisher(String, '/control_mode', 10)
        #self.posctrl_publisher = self.create_publisher(Bool, '/simple_drone/posctrl', 10) # position control setting 
        #self.dronevel_mode_publisher = self.create_publisher(Bool, '/simple_drone/dronevel_mode', 10) # velocity / tilt control
        #self.takeoff_publisher = self.create_publisher(Empty, '/simple_drone/takeoff', 10) # takeoff signal
        #self.landing_publisher = self.create_publisher(Empty, '/simple_drone/land', 10) # landing signal

        self.bridge = CvBridge()

        #uart constants
        self.SBUS_BAUD = 100000
        self.SBUS_HEADER = 0x0f 
        self.SBUS_FOOTER = 0x00

        self.SWITCHING_TIME_S = 1

        self.serial_port = serial.Serial(port = "/dev/ttyTHS1", bytesize=serial.EIGHTBITS, baudrate=100000, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_TWO)

        self.detector = pyyolo.YOLO('/home/patryk/darknet/cfg/yolov3-tiny.cfg', '/home/patryk/darknet/yolov3-tiny.weights', '/home/patryk/darknet/cfg/coco.data')
        #self.detector = pyyolo.YOLO('/home/patryk/darknet/cfg/yolov3.cfg', '/home/patryk/darknet/yolov3.weights', '/home/patryk/darknet/cfg/coco.data')

        #position variables
        self.det_x = IMG_H_C
        self.det_y = IMG_W_C
        self.det_s_queue = collections.deque(S_QUEUE_LEN*[TARG_SR*IMG_S], S_QUEUE_LEN)
        self.det_s = self.det_s_queue[0]

        #current control variables
        # self.pitch = 0
        # self.yaw = 0
        # self.roll = 0
        # self.throttle = 0
        self.curr_control_vals = np.zeros(4) #roll, pitch, throttle, yaw

        #target control variables
        # self.target_pitch = 0
        # self.target_yaw = 0	
        # self.target_roll = 0
        # self.target_throttle = 0
        self.target_control_vals = np.zeros(4) #roll, pitch, throttle, yaw

        #controllersself.throttle_controller.err_int
        self.roll_controller = Controller(IMG_W_C, 0.08, 10, 1.2, MIN_ROLL, MAX_ROLL) #x, d 0.005
        self.pitch_controller = Controller(TARG_SR*IMG_S, 0.07, float('inf'), 0.25, MIN_PITCH, MAX_PITCH) #S Td = 0.18
        self.throttle_controller = Controller(IMG_H_C, 0.6, 50, 5, MIN_THROTTLE, MAX_THROTTLE) #y 0.125
        # for tiny: pitch P = 0.08 is fine-ish

        #control vals
        self.control_msg = ManualControlSetpoint()
        self.control_msg.data_source = 2
        self.control_msg.valid = True
        self.control_msg.sticks_moving = True

        #switch vars
        self.switch_time = 0
        self.is_manual = True
        self.switch_start_vals = np.zeros(4)

        #create log file
        self.create_log()

    def img_callback(self, data):
        self.current_frame = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        # self.get_logger().info(f"Video frame received")

        #object detection
        dets = self.detector.detect(self.current_frame, rgb=False)

        self.det_x = IMG_W_C
        self.det_y = IMG_H_C
        self.det_s_queue.appendleft(TARG_SR*IMG_S)
        self.det_s = sum(self.det_s_queue)/S_QUEUE_LEN

        for i, det in enumerate(dets):
            if det.name == "stop sign":
                # self.get_logger().info(f"Detection: {i}, {det}")
                xmin, ymin, xmax, ymax = det.to_xyxy()

                self.det_x = int((xmin + xmax) / 2)
                self.det_y = int((ymin + ymax) / 2)
                self.det_s = ((xmax - xmin) * (ymax - ymin))**0.3

                self.current_frame = cv2.rectangle(self.current_frame, (xmin, ymin), (xmax, ymax), (0, 0, 255))
                break
        
        cv2.imshow('camera', self.current_frame)
        cv2.waitKey(1)

        #calculate and assign control vals
        # self.linear_comp.y = self.roll_controller.update(self.det_x)
        # self.linear_comp.z = self.throttle_controller.update(self.det_y)
        # self.linear_comp.x = self.pitch_controller.update(self.det_s)
        # self.control_msg.linear = self.linear_comp
        # self.control_msg.angular = self.angular_comp

        # print(f"{self.det_x}, {self.det_y}, {roll_val}, {self.throttle_controller.err_int}, {throttle_val}, {pitch_val}")

        #send control vals
        # self.control_publisher.publish(self.control_msg)
        # print(self.control_msg.linear)
        #update target control vals
        
        if not self.is_manual:
            self.target_control_vals[0] = -self.roll_controller.update(self.det_x)
            self.target_control_vals[2] = self.throttle_controller.update(self.det_y)
            self.target_control_vals[1] = self.pitch_controller.update(self.det_s)
        """if np.abs(self.roll_controller.err) > 0.7 or np.abs(self.throttle_controller.err) > 0.7:
            self.get_logger().info("reducing pitch")
            self.target_control_vals[1] = 0.5 * self.target_control_vals[1]
        """
       # self.get_logger().info(f"{self.target_control_vals[2]}")

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

            #manual_control_channels = manual_control_channels / (2**11) #convert to range -0.5, 0.5
            #manual_control_channels = manual_control_channels - (manual_control_channels/2)

            #POSSIBLE ISSUES WITH MANUAL
            #manual_control_channels = ((manual_control_channels - 172) - 819.5) / 409.75 #convert to range -2, 2
            manual_control_channels = ((manual_control_channels - 172) - 819.5) / 409.75 / 2 
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

        self.get_logger().info(f"{self.curr_control_vals, self.throttle_controller.err_int}")
        self.control_publisher.publish(self.control_msg)
        
        self.update_log()
       
        mode = "autonomous"
        if self.is_manual:
            mode = "manual"

        msg = String()
        msg.data = f"Control mode: {mode}"
        self.control_mode_publisher.publish(msg) 
        

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
        self.log.write(f"{time.time() - self.log_start_time} {self.det_x} {self.det_y} {self.det_s} {self.curr_control_vals[0]} {self.curr_control_vals[1]} {self.curr_control_vals[2]} {self.curr_control_vals[3]} {int(self.is_manual)}\n")  

    def set_control_mode(self, mode):
        #true - position control
        #false - normal mode

        msg = Bool()
        msg.data = mode
        self.posctrl_publisher.publish(msg)

    def set_dronevel_mode(self, mode):
        #true - velocity
        #false - tilt

        msg = Bool()
        msg.data = mode
        self.dronevel_mode_publisher.publish(msg)

    def takeoff(self):
        self.takeoff_publisher.publish(Empty())

    def land(self):
        self.landing_publisher.publish(Empty())


def main():
    rclpy.init()
    reent_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
    img_callback = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    timer_callback = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
    controller = Controller_Node(img_callback, timer_callback)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(controller)

    #init control mode and takeoff
    #controller.set_control_mode(False)
    #controller.set_dronevel_mode(False)
    #controller.takeoff()

    # rclpy.spin(controller)
    executor.spin()


    cv2.destroyAllWindows()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
