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
import rospy
import math
import copy
#from ctypes import c_ushort
from rover_msgs.msg import ArmState, Feedback, DynamixelState
from rover_msgs.srv import PositionReturn
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import String,Float32MultiArray,UInt16MultiArray, Header, Int8
import time
import numpy as np
from urdf_parser_py.urdf import URDF

from flask import Flask, render_template
from flash_ask import Ask, statement, question, session
import json
import requests
import sys
from junja2 import Template
import subprocess
#from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
#from pykdl_utils.kdl_kinematics import KDLKinematics
import random
import tf
import tf.transformations as tr

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

        self.state = ArmState()
        self.joints = JointState()
        #self.joints_cart = Float32MultiArray()
        self.pose_current = Pose()
        self.pose_cmd = Pose()
        self.grip = 0
        self.lock_grip = False

        # Initialize state; default = JointControl & Slow
        self.state.mode = 'JointControl' # 'JointControl', 'IK Arm - Base,Tool', 'IK Arm - Tool,Tool'
        self.state.speed = 'Slow' # Slow, Med, Fast
        self.state.kill = False

        # Initialize joints = instance of JointState
        self.joints.header = Header()
        self.joints.header.stamp = rospy.Time.now()
        self.joints.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joints.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joints.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joints.effort = []

        # Initialize FK
        self.init_ik = True
        self.pose_current.position.x = 0.0
        self.pose_current.position.y = 0.0
        self.pose_current.position.z = 0.0

        # Pre-set configurations
        self.rear_home = [-1.114, -2.123, -0.553, 0.0, 0.0, 0.0]
        self.chute_X = [2.060274156412888, -2.0234791766029856, -0.6918490064483838, -3.124139361069849, -0.308263710245888, -0.17661176511209503]
        self.chute_B = [2.0489237223000636, -3.1396775580652347, -1.4859738161057947, -3.124139361069849, 0.0, 0.0]
        self.clear = [-0.02417092222061374, -2.5997124416558117, -1.0112442595633835, -3.141592653589793, 0.0, 0.0]
        self.clamp = [2.6461728731994203, -3.0803258202728627, -1.7390465576449696, -3.124139361069849, -0.17890474362474218, 0.02768212655435109]

        #################################
        # Publishers and Subscribers
        #################################
        # Solver Answers
        self.sub_pose_cmd= rospy.Subscriber('/pose_cmd_ik', Pose, self.ikPoseCallback)
        self.sub_joint_cmd_ik = rospy.Subscriber('/joint_cmd_ik',JointState, self.ikJointCallback)

        # Useful GUI info
        self.pub_state = rospy.Publisher('/arm_state_cmd', ArmState, queue_size = 10)
        # Info sent to psoc.py and DynamixelPublisher.py about Joint positions
        self.pub_joints = rospy.Publisher('/joint_cmd', JointState, queue_size = 10)
        # Solver Inputs
        self.pub_joint_ik = rospy.Publisher('/joint_ik', JointState, queue_size = 10)
        self.pub_pose_ik = rospy.Publisher('/pose_ik', Pose, queue_size = 10)
        # Info sent to psoc.py about claw open/close
        self.pub_grip = rospy.Publisher('/grip', Int8, queue_size = 10)

        self.trigger_init = {'left': False, 'right': False}

        self.arm_init = False

        self.ready_msg = False

        self.getArmPosition()

    def ikJointCallback(self, msg):
        #for i in range(0,6):
        #    self.joints.position[i] = msg.position[i]
        #print "JOINT POSITIONS LENGTH FOR IK" + len(msg.position)
        # Map turret
        self.joints.position[0] = self.remap(msg.position[0], math.radians(-90),   math.radians(180),  -math.pi, math.pi)
        self.joints.position[1] = self.remap(msg.position[1], math.radians(-5),    math.radians(90),   -math.pi, math.pi)
        self.joints.position[2] = self.remap(msg.position[2], math.radians(-45),   math.radians(60),   -math.pi, math.pi)
        self.joints.position[3] = self.remap(msg.position[3], math.radians(-170),  math.radians(170),  -math.pi, math.pi)
        self.joints.position[4] = self.remap(msg.position[4], math.radians(-80),   math.radians(70),   -math.pi, math.pi)
        self.joints.position[5] = self.remap(msg.position[5], math.radians(-190),  math.radians(190),  -math.pi, math.pi)
        #print self.joints
        self.pub_joints.publish(self.joints)

    def ikPoseCallback(self,msg):
        self.pose_current = msg
    
    def getArmPosition(self):
        if not self.arm_init:
            rospy.loginfo("Waiting for Arm Position")
            try:
                rospy.wait_for_service('arm_position', 5)
                wristReq = rospy.ServiceProxy('arm_position', PositionReturn)
                wristResp = wristReq()
                self.joints.position[0] = wristResp.position[0]
                self.joints.position[1] = wristResp.position[1]
                self.joints.position[2] = wristResp.position[2]
                self.joints.position[3] = wristResp.position[3]
                rospy.loginfo('Got Arm Position')
            except rospy.ServiceException, e:
                print("oh no")
            except Exception, e:
                rospy.logwarn('Arm could not find joints!!')
                self.joints.position[0] = 0.0
                self.joints.position[1] = 0.0
                self.joints.position[2] = 0.0
                self.joints.position[3] = 0.0
                self.joints.position[4] = 0.0
                self.joints.position[5] = 0.0

    def gimbalStateCallback(self, msg):
        self.gimbal_read_pan = msg.pan

    def gimbal_reset(self):
        self.gimbal_cmd.pan = self.PAN_HOME
        self.gimbal_cmd.tilt = self.TILT_HOME
        self.pub_gimbal.publish(self.gimbal_cmd)

    def nod(self):
        rospy.logwarn('YEET')
        angle = radians(30)

        rospy.logwarn('LOOK AROUND')
        self.gimbal_reset()
        leftright_angle = radians(90)
        for i in range(4):
            leftright_angle = -leftright_angle
            self.gimbal_cmd.pan = leftright_angle
            self.pub_gimbal.publish(self.gimbal_cmd)
            rospy.sleep(0.8)
        rospy.sleep(1.0)

        rospy.logwarn('NOD HEAD')
        self.gimbal_reset()
        for i in range(8):
            angle = -angle
            self.gimbal_cmd.tilt = angle
            self.pub_gimbal.publish(self.gimbal_cmd)
            rospy.sleep(0.2)
        self.gimbal_cmd.tilt = 0
        self.pub_gimbal.publish(self.gimbal_cmd)
        rospy.sleep(0.1)
        self.gimbal_reset()
        rospy.sleep(1.0)
    
    def resetArm(self):
        joints_start = [-1.513, -1.8555, -3.050, 0, 0.33, -1.680]
        self.joints.position = joints_start
        self.pub_joints.publish(self.joints)
        rospy.sleep(1.0)

    def shake(self):
        joints_start = [-1.513, -1.8555, -3.050, 0, 0.33, -1.680]
        shake_start = [-1.485, -2.973, -2.583, 0, 0.649, -0.815]
        shake_wrist_top = 0.958
        shake_wrist_bottom = 0.279

        self.joints.position = joints_start
        self.pub_joints.publish(self.joints)
        rospy.sleep(1.0)

        self.state.speed = 'Med'
        self.joints.position = shake_start
        self.pub_joints.publish(self.joints)
        self.grip = 100
        self.pub_grip.publish(self.grip)
        rospy.sleep(2.5)
        self.grip = 0
        self.pub_grip.publish(self.grip)
        rospy.sleep(1.0)

        self.grip = -100
        self.pub_grip.publish(self.grip)
        rospy.sleep(1.0)
        self.grip = 0
        self.pub_grip.publish(self.grip)
        rospy.sleep(0.5)

        flip = False
        for i in range(8):
            flip = not flip
            if flip:
                self.joints.position[4] = shake_wrist_top
            else:
                self.joints.position[4] = shake_wrist_bottom
            self.pub_joints.publish(self.joints)
            rospy.sleep(0.2)
        self.grip = 100
        self.pub_grip.publish(self.grip)
        rospy.sleep(1.0)
        self.grip = 0
        self.pub_grip.publish(self.grip)

    @ask.launch
    def launch():
        print('launching...')
        return statement('launch')
    
    @ask.intent('HelloIntent')
    def hello():
        return statement('hi')

    @ask.intent('NodIntent')
    def nodIntent():
        pass

    @ask.intent('ShakeIntent')
    def shakeIntent():
        pass

if __name__ == '__main__':
    rospy.init_node('nod_head')

    print("initing new tricks class...")
    tricks = Tricks()
    print("done")
    print("reset motor positions...")
    tricks.resetArm()
    tricks.gimbal_reset()
    print("done")

    print("setting up flask ask server...")
    app = Flask(__name__, template_folder='template')
    ask = Ask(app, '/')
    print("done")

    print("starting server...")
    app.run(debug=True, host='0,0,0,0', port=8000)
    
