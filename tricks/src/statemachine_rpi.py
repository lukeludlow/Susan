#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from rover_msgs.msg import WaypointNav, NavStatus
# trick modules
from xboxarm import Arm_XBOX
from shake import Shake
from nod import Nod

class StateMachine:
    def __init__(self):
        #rospy.init_node('state_machine')
        self.rate = rospy.Rate(60)  # set rate to 60 hz
        # publishers
        rospy.loginfo('state machine')

if __name__ == '__main__':
    rospy.init_node('automaton')
    rospy.loginfo('###')
    rospy.loginfo('### automaton running!')
    rospy.loginfo('###')

    # trick objects    
    SM = StateMachine()
    xboxarm = Arm_XBOX()

    nod_trick = Nod()
    nod_trick.nod()
    rospy.sleep(10.0)
    shake_trick = Shake()
    shake_trick.shake() 

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