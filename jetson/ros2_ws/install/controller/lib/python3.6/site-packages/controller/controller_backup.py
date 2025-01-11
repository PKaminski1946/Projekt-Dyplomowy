import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import numpy as np
import pyyolo

from std_msgs.msg import String, Bool, Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3

#camera image size
IMG_H = 360
IMG_W = 640

IMG_H_C = IMG_H / 2
IMG_W_C = IMG_W / 2
IMG_S = IMG_H * IMG_W

#target object to image ratio
TARG_SR = 0.07

#max control vals, implemet this in the controller class as a parameter
MAX_ROLL = 1
MIN_ROLL = -1
MAX_PITCH = 1
MIN_PITCH = -1
MAX_THROTTLE = 1
MIN_THROTTLE = -1

class Controller():
    def __init__(self, sp ,kp, ki, kd, min_control_val, max_control_val):
        self.sp = sp
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_con = min_control_val
        self.max_con = max_control_val

        self.err_int = 0
        self.prev_err = 0

    def update(self, x):
        #change this to include min and max vals
        err = (self.sp - x) / self.sp #standardize err to [-1, 1]

        self.err_int += err
        self.err_int = np.clip(self.err_int, self.min_con, self.max_con)

        y = self.kp*err + self.ki*self.err_int + self.kd*(err - self.prev_err)

        self.prev_err = err

        # if y > 0:
        #     return y * self.max_con

        # elif y < 0:
        #     return y * self.min_con

        return y

class Controller_Node(Node):
    def __init__(self):
        super().__init__('controller')

        self.subscription = self.create_subscription(Image, '/simple_drone/front/image_raw', self.img_callback, 10) # raw camera img
        self.subscription
        
        self.control_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10) # movement control
        self.posctrl_publisher = self.create_publisher(Bool, '/simple_drone/posctrl', 10) # position control setting 
        self.dronevel_mode_publisher = self.create_publisher(Bool, '/simple_drone/dronevel_mode', 10) # velocity / tilt control
        self.takeoff_publisher = self.create_publisher(Empty, '/simple_drone/takeoff', 10) # takeoff signal
        self.landing_publisher = self.create_publisher(Empty, '/simple_drone/land', 10) # landing signal

        self.bridge = CvBridge()

        # self.detector = pyyolo.YOLO('/home/patryk/darknet/cfg/yolov3-tiny.cfg', '/home/patryk/darknet/yolov3-tiny.weights', '/home/patryk/darknet/cfg/coco.data')
        self.detector = pyyolo.YOLO('/home/patryk/darknet/cfg/yolov3.cfg', '/home/patryk/darknet/yolov3.weights', '/home/patryk/darknet/cfg/coco.data')

        #position variables
        self.det_x = None
        self.det_y = None

        #control variables
        self.pitch = 0
        self.yaw = 0
        self.roll = 0
        self.throttle = 0

        #controllers
        self.roll_controller = Controller(IMG_W_C, 0.2, 0, 0.02, MIN_ROLL, MAX_ROLL) #x
        self.pitch_controller = Controller(TARG_SR*IMG_S, 0.2, 0, 0.1, MIN_PITCH, MAX_PITCH) #S
        self.throttle_controller = Controller(IMG_H_C, 0.2, 0, 0.02, MIN_THROTTLE, MAX_THROTTLE) #y

        #control vals
        self.control_msg = Twist()
        self.linear_comp = Vector3()
        self.angular_comp = Vector3()

        self.linear_comp.x = 0.0
        self.linear_comp.y = 0.0
        self.linear_comp.z = 0.0

        self.angular_comp.x = 0.0
        self.angular_comp.y = 0.0
        self.angular_comp.z = 0.0



    def img_callback(self, data):
        self.current_frame = self.bridge.imgmsg_to_cv2(data, 'rgb8')
        # self.get_logger().info(f"Video frame received")

        #object detection
        dets = self.detector.detect(self.current_frame, rgb=False)

        self.det_x = IMG_W_C
        self.det_y = IMG_H_C
        self.det_s = TARG_SR*IMG_S

        for i, det in enumerate(dets):
            if det.name == "stop sign":
                # self.get_logger().info(f"Detection: {i}, {det}")
                xmin, ymin, xmax, ymax = det.to_xyxy()

                self.det_x = int((xmin + xmax) / 2)
                self.det_y = int((ymin + ymax) / 2)
                self.det_s = (xmax - xmin) * (ymax - ymin)

                self.current_frame = cv2.rectangle(self.current_frame, (xmin, ymin), (xmax, ymax), (0, 0, 255))
                break
        
        cv2.imshow('camera', self.current_frame)
        cv2.waitKey(1)

        #calculate and assign control vals
        self.linear_comp.y = self.roll_controller.update(self.det_x)
        self.linear_comp.z = self.throttle_controller.update(self.det_y)
        self.linear_comp.x = self.pitch_controller.update(self.det_s)
        self.control_msg.linear = self.linear_comp
        self.control_msg.angular = self.angular_comp

        # print(f"{self.det_x}, {self.det_y}, {roll_val}, {self.throttle_controller.err_int}, {throttle_val}, {pitch_val}")

        #send control vals
        self.control_publisher.publish(self.control_msg)
        print(self.control_msg.linear)
        
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

    controller = Controller_Node()

    #init control mode and takeoff
    controller.set_control_mode(False)
    controller.set_dronevel_mode(False)
    controller.takeoff()

    rclpy.spin(controller)

    cv2.destroyAllWindows()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()