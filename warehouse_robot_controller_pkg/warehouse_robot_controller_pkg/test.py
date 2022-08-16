import numpy as np

scan_array = np.ones((180))
left_dist = np.sum(scan_array[72:108])

d_l = np.sum(scan_array[144:181])
d_lf = np.sum(scan_array[108:144])
d_f = np.sum(scan_array[72:108])
d_fr = np.sum(scan_array[36:72])
d_r = np.sum(scan_array[0:36])

def follow_wall(self):
    msg = Twist()
    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    # Logic for following the wall
    # >d means no wall detected by that laser beam
    # <d means an wall was detected by that laser beam
    d = self.d_thresh

    if self.d_lf > d and self.d_f > d and self.d_fr > d:
        self.wall_following_state = "search for wall"
        msg.linear.x = self.v_f
        msg.angular.z = -self.w_min  # turn right to find wall

    elif self.d_lf > d and self.d_f < d and self.d_fr > d:
        self.wall_following_state = "turn left"
        msg.angular.z = self.w_max

    elif (self.d_lf > d and self.d_f > d and self.d_fr < d):
        if (self.d_fr < self.d_collision):
            # Getting too close to the wall
            self.wall_following_state = "turn left"
            msg.linear.x = self.v_f
            msg.angular.z = self.w_max
        else:
            # Go straight ahead
            self.wall_following_state = "follow wall"
            msg.linear.x = self.v_f

    elif self.d_lf < d and self.d_f > d and self.d_fr > d:
        self.wall_following_state = "search for wall"
        msg.linear.x = self.v_f
        msg.angular.z = -self.w_min  # turn right to find wall

    elif self.d_lf > d and self.d_f < d and self.d_fr < d:
        self.wall_following_state = "turn left"
        msg.angular.z = self.w_max

    elif self.d_lf < d and self.d_f < d and self.d_fr > d:
        self.wall_following_state = "turn left"
        msg.angular.z = self.w_max

    elif self.d_lf < d and self.d_f < d and self.d_fr < d:
        self.wall_following_state = "turn left"
        msg.angular.z = self.w_max

    elif self.d_lf < d and self.d_f > d and self.d_fr < d:
        self.wall_following_state = "search for wall"
        msg.linear.x = self.v_f
        msg.angular.z = -self.w_min  # turn right to find wall

    else:
        pass

    # Send velocity command to the robot
    self.publisher_.publish(msg)


print(left_dist)