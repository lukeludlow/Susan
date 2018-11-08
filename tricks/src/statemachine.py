#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from rover_msgs.msg import WaypointNav, NavStatus
from wheel_controller import WheelController

class StateMachine:
    def __init__(self):
        rospy.init_node('state_machine')
        self.rate = rospy.Rate(60)  # set rate to 60 hz
        # publishers
        self.pub_drive = rospy.Publisher('/auto_vel', Twist, queue_size=10)
        self.status_timer = rospy.Timer(rospy.Duration(.1), self.statusPubCallback)
        rospy.loginfo('state machine running')

    def execute(self):
        while not rospy.is_shutdown():
            self.check_states()
            self.send_command(v, w)
            self.rate.sleep()  # execute at specified rate
        self.status_timer.shutdown()

    # check for transitions between states
    def check_states(self):
        #  TODO
        """
        if self.controller == self.gotogoal:
            self.process_go_to_goal()

        elif self.controller == self.obstacle_avoidance:
            self.process_obstacle()

        elif self.controller == self.approachball:
            self.process_tennis_ball()
        """

    # send velocities to wheel_controller
    def send_wheel_command(self, v, w):
        cmd_msg = Twist()
        cmd_msg.linear.x = v
        cmd_msg.angular.z = w
        self.pub_drive.publish(cmd_msg)

    def reset(self):
        self.auto_enable = False
        self.controller.status = self.controller.default_status


if __name__ == '__main__':
    rospy.init_node('automaton')
    rospy.loginfo('##################')
    rosply.loginfo('automaton running')
    rospy.loginfo('##################')
    self.rate = rospy.Rate(60)  # set rate to 60 hz
    # publishers
    self.pub_drive = rospy.Publisher('/auto_vel', Twist, queue_size=10)
    self.status_timer = rospy.Timer(rospy.Duration(.1), self.statusPubCallback)
    
    SM = StateMachine()

    rate = rospy.Rate(60)  # set rate to 60 hz

    # execute automaton
    while not rospy.is_shutdown():
        # Start when Xbox controller recognized
        if len(xbox.joy.buttons) > 0 and xbox.trigger_check():
            if not xbox.ready_msg:
                rospy.loginfo('Arm Controller Ready')
                xbox.ready_msg = True
            # every time check toggle of state
            xbox.check_method()
            # check for kill switch (True = Killed)
            if xbox.state.kill  == False:
                # call appropriate function for state
                # defaults to JointControl
                if xbox.state.mode == 'JointControl':
                    xbox.joint_cmd()
                elif xbox.state.mode == 'IK Arm - Base,Tool':
                    xbox.arm_IK_base_tool()
                elif xbox.state.mode == 'IK Arm - Tool,Tool':
                    xbox.arm_IK_tool_tool()
                else:
                    xbox.joint_cmd()
        rate.sleep()