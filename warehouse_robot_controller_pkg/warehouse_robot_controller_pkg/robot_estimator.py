import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray
import numpy as np


class Estimator(Node):

    def __init__(self):

        super().__init__('Estimator')
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/demo/odom',
            self.odom_callback,
            10)

        self.velocity_subscriber = self.create_subscription(
            Twist,
            '/demo/cmd_vel',
            self.velocity_callback,
            10)

        self.publisher_state_est = self.create_publisher(
            Float64MultiArray,
            '/demo/state_est',
            10)

    def odom_callback(self, msg):
        """
        Receive the odometry information containing the position and orientation
        of the robot in the global reference frame. 
        The position is x, y, z.
        The orientation is a x,y,z,w quaternion. 
        """
        roll, pitch, yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        obs_state_vector_x_y_yaw = [
            msg.pose.pose.position.x, msg.pose.pose.position.y, yaw]

        self.publish_estimated_state(obs_state_vector_x_y_yaw)

    def publish_estimated_state(self, state_vector_x_y_yaw):
        """
        Publish the estimated pose (position and orientation) of the 
        robot to the '/demo/state_est' topic. 
        :param: state_vector_x_y_yaw [x, y, yaw] 
        x is in meters, y is in meters, yaw is in radians
        """
        msg = Float64MultiArray()
        msg.data = state_vector_x_y_yaw
        self.publisher_state_est.publish(msg)

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def velocity_callback(self, msg):
        """
        Listen to the velocity commands (linear forward velocity 
        in the x direction in the robot's reference frame and 
        angular velocity (yaw rate) around the robot's z-axis.
        [v,yaw_rate]
        [meters/second, radians/second]
        """
        v = msg.linear.x
        yaw_rate = msg.angular.z


def main(args=None):
    """
    Entry point for the program.
    """
    rclpy.init(args=args)
    estimator = Estimator()
    rclpy.spin(estimator)
    estimator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
