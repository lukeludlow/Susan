#!/usr/bin/python
from __future__ import print_function
import numpy as np
import rospy
from math import radians, degrees
from Controller import Controller
from PID import PID
from rover_msgs.msg import TennisLocation, DynamixelState
from std_srvs.srv import Empty
from rover_msgs.srv import DistHeadingConv, WaypointSend

class Tricks(Controller):
    def __init__(self):
        Controller.__init__(self)
        self.name = "tricks"

        # Constants
        self.BALL_W = 2.3   # rad/s, rover frame
        self.BALL_V = 1.8  # m/s
        self.PROB_THRESH = 0.5  # For positive tennis ball detection
        self.FOLLOW_PROB_THRESH = 0.35  # Lower threshold during center_cam_and_drive
        self.PAN_HOME = 1  # 1 radian is actually zero position for pan
        self.PAN_START = radians(207)  # 207 is facing backwards-left, -90 is backwards-right
        self.PAN_END = radians(-90)
        self.PAN_SPEED = 0.25  # speed of pan from left to right [rad/s]
        tilt_offset = radians(20)
        self.TILT_START = radians(80) + tilt_offset
        self.TILT_MID = radians(45) + tilt_offset
        self.TILT_HOME = radians(20) + tilt_offset
        self.TILT_SHORT_START = radians(40) + tilt_offset
        self.PAN_SHORT_START = self.PAN_HOME + radians(45)
        self.PAN_SHORT_END = self.PAN_HOME - radians(45)
        self.pan_start = self.PAN_START
        self.pan_end = self.PAN_END
        self.BALL_DIST_THRESH = 2. # m
        self.WAYPOINT_THRESH = 4.  # m
        self.ESTIMATE_DISTANCE = 7.0 # m
        self.ESTIMATE_HEADING_THRESH = 45 # deg
        self.BALL_HEADING_THRESH = 10  # deg
        self.LOST_THRESH = 120  # counts in lost_counter

        self.pid = PID(kp=4.5, ki=0, kd=0.05, lim=self.BALL_W)

        # Flags
        self.found_ball = False
        self.search_init = False
        self.exit = False
        self.arrived_at_ball = False
        self.estimate_ball = False

        # Initializing
        self.lost_count = 0
        self.ball_prob = 0.0
        self.tb_dist = np.inf
        self.ball_center = [0,0]
        self.image_center = [0,0]

        self.gimbal_cmd = DynamixelState()
        self.gimbal_cmd.pan = self.PAN_HOME
        self.gimbal_cmd.tilt = self.TILT_HOME
        self.gimbal_read_pan = 0

        self.final_waypoint = []
        self.search_pattern_init = [[15,45],[15,-45],[15,-135],[15,135]] # ditance (m), heading (deg from North)
        self.search_waypoints = [] # start empty
        self.waypoint_count = 0

        # Subscribers
        self.sub_gimbal_state = rospy.Subscriber('/gimbal_state', DynamixelState, self.gimbalStateCallback)

        # Publishers
        self.pub_gimbal = rospy.Publisher('/gimbal_cmd_position', DynamixelState, queue_size = 1)


        #self.nod()
        #self.gimbal_reset()


    def gimbalStateCallback(self, msg):
        self.gimbal_read_pan = msg.pan


    def nod(self):
        rospy.logwarn('YEET')

        rospy.logwarn('LOOK AROUND')
        self.gimbal_reset()
        rospy.sleep(0.2)

        init_angle = radians(180)
        self.gimbal_cmd.pan = init_angle
        self.pub_gimbal.publish(self.gimbal_cmd)
        rospy.sleep(2.5)
        up_angle = radians(1)
        self.gimbal_cmd.tilt = -up_angle
        self.pub_gimbal.publish(self.gimbal_cmd)
        rospy.sleep(1.0)
        rospy.logwarn('NOD HEAD')
        self.gimbal_reset()
        rospy.sleep(1.0)
        angle = radians(30)
        for i in range(8):
            angle = -angle
            self.gimbal_cmd.tilt = angle
            self.pub_gimbal.publish(self.gimbal_cmd)
            rospy.sleep(0.2)
        self.pub_gimbal.publish(self.gimbal_cmd)
        rospy.sleep(0.2)
        self.gimbal_reset()
        rospy.sleep(1.0)

    def gimbal_reset(self):
        self.gimbal_cmd.pan = self.PAN_HOME
        self.gimbal_cmd.tilt = self.TILT_HOME
        self.pub_gimbal.publish(self.gimbal_cmd)

if __name__ == '__main__':
    rospy.init_node('nod_head')
    tricks = Tricks()
    tricks.nod()
    
