import math
import rclpy
from time import sleep
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data
import numpy as np


class Controller(Node):
    def __init__(self):

        super().__init__('Controller')

        self.subscription = self.create_subscription( Float64MultiArray, '/demo/state_est', self.state_estimate_callback, 10)
        self.subscription  # prevent unused variable warning

        self.scan_subscriber = self.create_subscription( LaserScan, '/demo/laser/out', self.scan_callback, qos_profile=qos_profile_sensor_data)

        self.publisher_ = self.create_publisher( Twist, '/demo/cmd_vel', 10)

        self.d_l = 999999.9  # Left
        self.d_lf = 999999.9  # Left-front
        self.d_f = 999999.9  # Front
        self.d_fr = 999999.9  # Right-front
        self.d_r = 999999.9  # Right
        self.scan_array = np.zeros((181))

        self.v_f = 0.025
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.wall_following_state = "turn left"

        self.w_max = 3.0  # Fast turn
        self.w_min = 0.05  # Slow turn

        self.d_thresh = 0.50  # in meters

        self.d_collision = 0.19  # in meters

    def state_estimate_callback(self, msg):
        
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]

        self.run_midway()

    def scan_callback(self, msg):

        for i in range(len(self.scan_array)):
            self.scan_array[i] = msg.ranges[i]
        
        # Average sum of laser scan over range of sections

        self.d_l = np.sum(self.scan_array[144:181])
        self.d_lf = np.sum(self.scan_array[108:144])
        self.d_f = np.sum(self.scan_array[72:108])
        self.d_fr = np.sum(self.scan_array[36:72])
        self.d_r = np.sum(self.scan_array[0:36])

    def run_midway(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        # Simple Logic for path planning

        vx = self.d_f + (self.d_lf - self.d_fr)*math.cos(math.pi/4) 
        vy = self.d_r - self.d_l + (self.d_lf - self.d_fr)*math.cos(math.pi/4)
        w = - (vy/vx) 

        msg.linear.x = vx/10000
        msg.linear.y = - vy/10000
        msg.angular.z = w

        self.publisher_.publish(msg)

def main(args=None):

    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
