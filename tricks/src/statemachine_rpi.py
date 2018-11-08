#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from rover_msgs.msg import WaypointNav, NavStatus
# trick modules
from xboxarm import Arm_XBOX
import shake
import nod

class StateMachine:
    def __init__(self):
        #rospy.init_node('state_machine')
        self.rate = rospy.Rate(60)  # set rate to 60 hz
        # publishers
        rospy.loginfo('state machine')

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
    rospy.loginfo('###')
    rospy.loginfo('### automaton running!')
    rospy.loginfo('###')

    # trick objects    
    SM = StateMachine()
    xboxarm = Arm_XBOX()

    # execute automaton
    rate = rospy.Rate(60)  # set rate to 60 hz
    while not rospy.is_shutdown():
        # Start when Xbox controller recognized
        if len(xboxarm.joy.buttons) > 0 and xboxarm.trigger_check():
            if not xboxarm.ready_msg:
                rospy.loginfo('Arm Controller Ready')
                xboxarm.ready_msg = True
            # every time check toggle of state
            xboxarm.check_method()
            # check for kill switch (True = Killed)
            if xboxarm.state.kill  == False:
                # call appropriate function for state
                # defaults to JointControl
                if xboxarm.state.mode == 'JointControl':
                    xboxarm.joint_cmd()
                elif xboxarm.state.mode == 'IK Arm - Base,Tool':
                    xboxarm.arm_IK_base_tool()
                elif xboxarm.state.mode == 'IK Arm - Tool,Tool':
                    xboxarm.arm_IK_tool_tool()
                else:
                    xboxarm.joint_cmd()
        rate.sleep()