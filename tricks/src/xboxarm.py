#!/usr/bin/env python

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
#from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
#from pykdl_utils.kdl_kinematics import KDLKinematics
import random
import tf
import tf.transformations as tr

class Arm_XBOX():
    def __init__(self):
    # Variables
        self.joy = Joy()
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
        # XBOX
        self.sub_joy = rospy.Subscriber('/joy_arm', Joy, self.joyCallback)
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


    ##### Callbacks ##########

    def joyCallback(self,msg):
        self.joy = msg

    def ikPoseCallback(self,msg):
        self.pose_current = msg


    def remap(self, value, minInput, maxInput, minOutput, maxOutput):
        value = maxInput if value > maxInput else value
        value = minInput if value < minInput else value
        inputSpan = maxInput - minInput
        outputSpan = maxOutput - minOutput
        scaled = float(value - minInput) / float(inputSpan)
        return minOutput + (scaled * outputSpan)

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

    # Functions
    def check_method(self):
        # Check to see whether driving or using arm and return case
        # [A, B, X, Y] = buttons[0, 1, 2, 3]
        y = self.joy.buttons[3] # Y button toggles between IK and FK modes
        home = self.joy.buttons[8] # XBox button (center button)
        A = self.joy.buttons[0] # A button resets wrist
        B = self.joy.buttons[1] # Resets forearm
        lb = self.joy.buttons[4]

        if y == 1 and False: # IK modes currently disabled (remove 'and False')
            # Joint Control is direct control of each joint (forward kinematics),
            # IK Arm - Base,Tool is inverse kinematics with position in base frame, orientation in tool frame
            # IK Arm - Tool,Tool is inverse kinematics with position and orientation in tool frame
            if self.state.mode == 'JointControl':
                self.state.mode = 'IK Arm - Base,Tool'
            elif self.state.mode == 'IK Arm - Base,Tool':
                self.state.mode = 'IK Arm - Tool,Tool'
            else:
                self.state.mode = 'JointControl'
            time.sleep(.25) # Delay to prevent over-actuation (only switch once on button press)
            rospy.loginfo(self.state.mode) # Print mode to the screen

        # Implement Kill Switch - button press alternates between off and on
        if home:
            if self.state.kill == False:
                self.state.kill  = True
            else:
                self.state.kill  = False
            time.sleep(.25)

        # A button is used to reset the wrist position quickly
        if lb and A:
            self.joints.position = copy.copy(self.rear_home)
            rospy.loginfo('Moving to Rear')
            time.sleep(0.25)
        elif A:
            self.joints.position[4] = 0
            self.joints.position[5] = 0
            time.sleep(.25)
            rospy.loginfo('Reset wrist')
        elif B:
            self.joints.position[3] = 0
            time.sleep(.25)
            rospy.loginfo('Reset forearm')

        # Publish state commands
        self.pub_state.publish(self.state)

        # Check for position commands
        self.programmed_positions()

    def programmed_positions(self):
        lb = self.joy.buttons[4]
        X = self.joy.buttons[2]
        B = self.joy.buttons[1]
        Y = self.joy.buttons[3]
        A = self.joy.buttons[0]
        rjoy_press = self.joy.buttons[10]

        hat_up = self.joy.axes[7]
        hat_right = self.joy.axes[6]

        # Left bumper is used to move arm directly to a given position
        if lb == 1:
            if X:
                self.joints.position = self.chute_X
                rospy.loginfo('Moving to Chute X')
                time.sleep(0.25)
            elif B:
                self.joints.position = self.chute_B
                rospy.loginfo('Moving to Chute B')
                time.sleep(0.25)
            elif A:
                self.joints.position = self.rear_home
                rospy.loginfo('Moving to Rear')
                time.sleep(0.25)
            elif Y:
                self.joints.position = self.clear
                rospy.loginfo('Clearing arm')
                time.sleep(0.25)
            elif rjoy_press:
                self.joints.position = self.clamp
                rospy.loginfo('Moving to Clamping position')
                time.sleep(0.25)
            elif hat_right ==  1 or hat_right == -1:   # Left Bumper + DPAD = rotate wrist 180 degrees. TODO: Change shortcut
            	self.joints.position[5] += np.pi*hat_right
            	time.sleep(0.25)

    def trigger_check(self):
        rt = (1 - self.joy.axes[5])/2.0
        lt = (1 - self.joy.axes[2])/2.0
        if rt == 1:
            self.trigger_init['right'] = True
        if lt == 1:
            self.trigger_init['left'] = True

        if self.trigger_init['left'] and self.trigger_init['right']:
            return True
        else:
            return False

    def speed_check(self):
        # Right bumper toggles between arm speeds
        rb = self.joy.buttons[5]
        if rb == 1:
            if self.state.speed == 'Slow':
                self.state.speed = 'Med'
            elif self.state.speed == 'Med':
                self.state.speed = 'Fast'
            elif self.state.speed == 'Fast':
                self.state.speed = 'Slow'
            time.sleep(.25)
            rospy.loginfo(self.state.speed)

    def gripper(self):
        rt = (1 - self.joy.axes[5])/2.0
        lt = (1 - self.joy.axes[2])/2.0
        X = self.joy.buttons[2]
        threshold = 0.1

        if rt >= threshold: # open
            self.lock_grip = False
            self.grip = rt*100
        elif lt >= threshold: # close
            if X:
                self.lock_grip = True
                self.grip = -100
                time.sleep(.5)
            elif not self.lock_grip:
                self.grip = -lt*100
        elif not self.lock_grip:
            self.grip = 0
        self.pub_grip.publish(self.grip)

    # ==========================================================================
    # INVERSE KINEMATICS CONTROL Position = Base Frame; Orientation = End effector frame
    # ==========================================================================
    def arm_IK_base_tool(self):

        # read in & initialize position of arm
        # if first time
        if self.init_ik:
            # Publish current joint position
            self.pub_joint_ik.publish(self.joints)
            time.sleep(.25)
            self.pose_cmd = self.pose_current
            self.init_ik = False
        # FK on last commanded angles

        ###### change pose with Xbox
        # Speed Check
        self.speed_check()

        # Set corresponding rate
        if self.state.speed == 'Fast':
            MAX_RATE = .01
        elif self.state.speed == 'Med':
            MAX_RATE = .001
        elif self.state.speed == 'Slow':
            MAX_RATE = .0001

        ANGLE_RATE = 5

        # Calculate how to command arm (position control)
        DEADZONE = 0.1

        # Set axes
        left_joy_up = self.joy.axes[1]
        left_joy_right = self.joy.axes[0]
        right_joy_up = self.joy.axes[4]
        right_joy_right = self.joy.axes[3]
        hat_up = self.joy.axes[7]
        hat_right = self.joy.axes[6]

        # make array of axes
        axes = [left_joy_right*-1, -1*left_joy_up, hat_up,
            hat_right, right_joy_up, right_joy_right]

        # Set axis to zero in deadzone
        for i in range(0,len(axes)):
            if abs(axes[i])<DEADZONE:
                axes[i] = 0

        # update pose_cmd with result from IK Node
        self.pose_cmd = self.pose_current

        # Update Cartesian Positions
        curRot = self.posemsg_to_rotmatrix(self.pose_cmd)
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

        self.pose_cmd.position.x += axes[0]*MAX_RATE
        self.pose_cmd.position.y -= axes[1]*MAX_RATE
        self.pose_cmd.position.z += axes[4]*MAX_RATE
        alpha = axes[2]*MAX_RATE*ANGLE_RATE
        beta = axes[5]*MAX_RATE*ANGLE_RATE
        gamma = axes[3]*MAX_RATE*ANGLE_RATE
        Rx = tf.transformations.rotation_matrix(alpha, xaxis)
        Ry = tf.transformations.rotation_matrix(beta,  yaxis)
        Rz = tf.transformations.rotation_matrix(gamma, zaxis)
        newRot = tf.transformations.concatenate_matrices(curRot,Rx,Ry,Rz)
        quat = tf.transformations.quaternion_from_matrix(newRot)

        self.pose_cmd.orientation.x = quat[0]
        self.pose_cmd.orientation.y = quat[1]
        self.pose_cmd.orientation.z = quat[2]
        self.pose_cmd.orientation.w = quat[3]

        # send pose to IK
        self.pub_pose_ik.publish(self.pose_cmd)


    # ==========================================================================
    # INVERSE KINEMATICS CONTROL 2 Position = End Effector Frame; Orientation = End effector frame
    # ==========================================================================
    def arm_IK_tool_tool(self):

        # read in & initialize position of arm
        # if first time
        if self.init_ik:
            # Publish current joint position
            self.pub_joint_ik.publish(self.joints)
            time.sleep(.25)
            self.pose_cmd = self.pose_current
            self.init_ik = False
        # FK on last commanded angles

        ###### change pose with Xbox
        # Speed Check
        self.speed_check()

        # Set corresponding rate
        if self.state.speed == 'Fast':
            MAX_RATE = .01
        elif self.state.speed == 'Med':
            MAX_RATE = .001
        elif self.state.speed == 'Slow':
            MAX_RATE = .0001

        ANGLE_RATE = 5

        # Calculate how to command arm (position control)
        DEADZONE = 0.1

        # Set axes
        left_joy_up = self.joy.axes[1]
        left_joy_right = self.joy.axes[0]
        right_joy_up = self.joy.axes[4]
        right_joy_right = self.joy.axes[3]
        hat_up = self.joy.axes[7]
        hat_right = self.joy.axes[6]

        # make array of axes
        axes = [left_joy_right*-1, left_joy_up, hat_up,
            hat_right, right_joy_up, right_joy_right]

        # Set axis to zero in deadzone
        for i in range(0,len(axes)):
            if abs(axes[i])<DEADZONE:
                axes[i] = 0

        # update pose_cmd with result from IK Node
        self.pose_cmd = self.pose_current

        ### Update Cartesian Position

        # Get Current Transformation to End Effector Tip
        curRot = self.posemsg_to_rotmatrix(self.pose_cmd)
        T = self.posemsg_to_matrix(self.pose_cmd)
        origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)

        # x,y,z movement in tool frame
        x_mvnt = axes[0]*MAX_RATE
        y_mvnt = -axes[1]*MAX_RATE
        z_mvnt = axes[4]*MAX_RATE
        mvnt = np.matrix([x_mvnt, y_mvnt, z_mvnt])
        dTool = tr.translation_matrix(mvnt)
        dBase = tr.concatenate_matrices(T, dTool)

        self.pose_cmd.position.x = dBase.item(0, 3)
        self.pose_cmd.position.y = dBase.item(1, 3)
        self.pose_cmd.position.z = dBase.item(2, 3)

        alpha = axes[2]*MAX_RATE*ANGLE_RATE
        beta = axes[5]*MAX_RATE*ANGLE_RATE
        gamma = axes[3]*MAX_RATE*ANGLE_RATE
        Rx = tr.rotation_matrix(alpha, xaxis)
        Ry = tr.rotation_matrix(beta,  yaxis)
        Rz = tr.rotation_matrix(gamma, zaxis)
        newRot = tr.concatenate_matrices(curRot, Rx, Ry, Rz)
        quat = tr.quaternion_from_matrix(newRot)

        self.pose_cmd.orientation.x = quat[0]
        self.pose_cmd.orientation.y = quat[1]
        self.pose_cmd.orientation.z = quat[2]
        self.pose_cmd.orientation.w = quat[3]

       # print self.pose_cmd.position.x, -axes[0]*MAX_RATE, left_joy_right

        # send pose to IK
        self.pub_pose_ik.publish(self.pose_cmd)

        #print self.pose_cmd

    def posemsg_to_matrix(self,posemsg):
        R = self.posemsg_to_rotmatrix(posemsg)
        T = self.posemsg_to_transmatrix(posemsg)
        H = tr.concatenate_matrices(T,R)
        return H

    def posemsg_to_rotmatrix(self,posemsg):
        quaternion = (
            posemsg.orientation.x,
            posemsg.orientation.y,
            posemsg.orientation.z,
            posemsg.orientation.w)
        H = tr.quaternion_matrix(quaternion)
        return H

    def posemsg_to_transmatrix(self,posemsg):
        trans = np.matrix([posemsg.position.x,posemsg.position.y,posemsg.position.z])
        H = tr.translation_matrix(trans)
        return H





    # ==========================================================================
    # Xbox Arm Control ===============================================
    # ==========================================================================
    def joint_cmd(self):

        # Speed Check
        self.speed_check()

        # Set corresponding rate
        if self.state.speed == 'Fast':
            MAX_RATE = math.radians(.5)
        elif self.state.speed == 'Med':
            MAX_RATE = math.radians(0.25)
        elif self.state.speed == 'Slow':
            MAX_RATE = math.radians(0.1)

        # Calculate how to command arm (position control)
        DEADZONE = 0.2

        # Set axes
        left_joy_up = self.joy.axes[1]
        left_joy_right = self.joy.axes[0]
        right_joy_up = self.joy.axes[4]
        right_joy_right = self.joy.axes[3]
        hat_up = self.joy.axes[7]
        hat_right = self.joy.axes[6]

        # print "\n\nArm:"


        # make array of axes
        axes = [left_joy_right*-1, left_joy_up, hat_up,
            hat_right*-1, right_joy_up, right_joy_right]
        # Set axis to zero in deadzone
        for i in range(0,len(axes)):
            if abs(axes[i])<DEADZONE:
                axes[i] = 0

        # print "Axes: ",
        # print axes
        # print self.joints.position

        # Update joint angles
        for i in range(0,6):
            # if i == 3:
                # self.joints.position[i] += axes[i]*math.radians(0.25)
            # else:
            self.joints.position[i] += axes[i]*MAX_RATE

        # print "Update: ",
        # print self.joints.position
        # Set joint angle limits
        for i in range(0,len(self.joints.position)):
           if self.joints.position[i] > np.pi:
               self.joints.position[i] = np.pi
           elif self.joints.position[i] < -np.pi:
               self.joints.position[i] = -np.pi
        # print "Limit: ",
        # print self.joints.position
        #wristlimit1 = math.radians(80)
        #if self.joints.position[4] > wristlimit1:
        #    self.joints.position[4] = wristlimit1
        #elif self.joints.position[4] < -wristlimit1:
        #    self.joints.position[4] = -wristlimit1


        self.joints.header.stamp = rospy.Time.now()
        self.joints.header.frame_id = 'JointControl'

        # Gripper
        self.gripper()

        # set flag so IK knows must init when entered again
        self.init_ik = True

        # Publish arm commands
        self.pub_joints.publish(self.joints)
        self.pub_joint_ik.publish(self.joints)

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