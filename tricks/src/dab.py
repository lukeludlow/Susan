#!/usr/bin/python
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
import random
import tf
import tf.transformations as tr

class Dab(Controller):
    def __init__(self):
        Controller.__init__(self)
        self.name = "dab"

        # HEAD STUFF
        # constants
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
        # initialize head gimble
        self.gimbal_cmd = DynamixelState()
        self.gimbal_cmd.pan = self.PAN_HOME
        self.gimbal_cmd.tilt = self.TILT_HOME
        self.gimbal_read_pan = 0
        # subscribers and publishers
        self.sub_gimbal_state = rospy.Subscriber('/gimbal_state', DynamixelState, self.gimbalStateCallback)
        self.pub_gimbal = rospy.Publisher('/gimbal_cmd_position', DynamixelState, queue_size = 1)
        

        # ARM STUFF
        self.state = ArmState()
        self.joints = JointState()
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
        # arm pre-set configurations
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

    # head helper functions        
    def gimbalStateCallback(self, msg):
        self.gimbal_read_pan = msg.pan
    def gimbal_reset(self):
        self.gimbal_cmd.pan = self.PAN_HOME
        self.gimbal_cmd.tilt = self.TILT_HOME
        self.pub_gimbal.publish(self.gimbal_cmd)

    # arm helper functions
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
                print wristResp.position 
            except rospy.ServiceException, e:  
                print "Service call failed: %s" %e
            except Exception, e:
                rospy.logwarn('Arm could not find joints!!')
                self.joints.position[0] = 0.0
                self.joints.position[1] = 0.0
                self.joints.position[2] = 0.0
                self.joints.position[3] = 0.0
                self.joints.position[4] = 0.0
                self.joints.position[5] = 0.0


    def dab(self):
        
        init_pos = [-1.400, -2.600, -2.500, self.joints.position[3], self.joints.position[4], self.joints.position[5]]
        dab_startpos = [-2.400, -2.020, 0.081, self.joints.position[3], self.joints.position[4], self.joints.position[5]]
        dab_endpos = [-2.800, -2.02, 0.666, self.joints.position[3], self.joints.position[4], self.joints.position[5]]

        rospy.logwarn('WARNING: FLEX ALERT')

        self.state.speed = 'Med'
        self.joints.position = init_pos
        self.pub_joints.publish(self.joints)
        rospy.sleep(2.0)

        # move arm to dab position
        self.state.speed = 'Med'
        self.joints.position = dab_startpos
        self.pub_joints.publish(self.joints)
        rospy.sleep(2.0)

        # turn head around
        self.gimbal_reset()
        rospy.sleep(1.0)
        turnaround_angle = radians(360)
        self.gimbal_cmd.pan = turnaround_angle
        self.pub_gimbal.publish(self.gimbal_cmd)
        rospy.sleep(1.5)

        # set head down pos
        dab_angle = radians(40)
        self.gimbal_cmd.tilt = dab_angle
        # set dab flick position
        self.state.speed = 'Fast'
        self.joints.position = endpos

        # perform the dab flick
        rospy.loginfo("FLEX")
        self.pub_gimbal.publish(self.gimbal_cmd)
        self.pub_joints.publish(self.joints)
        rospy.sleep(1.0)

        # flick back to normal dab position
        self.state.speed = 'Fast'
        self.joints.position = dab_startpos
        self.pub_joints.publish(self.joints)
        self.gimbal_reset()
        rospy.sleep(1.0)

        # reset positions
        self.state.speed = 'Med'
        self.joints.position = init_pos
        self.pub_joints.publish(self.joints)
        rospy.sleep(2.0)
