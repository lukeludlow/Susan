#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse, Trigger, TriggerResponse
from rover_msgs.msg import WaypointNav, NavStatus, Temperature
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

def trickCallback(msg):
    rospy.loginfo('trickCallback')
    return msg.average_temperature

if __name__ == '__main__':
    rospy.init_node('automaton')
    rospy.loginfo('###')
    rospy.loginfo('### automaton running!')
    rospy.loginfo('###')
    sub = rospy.Subscriber('/tricks', Temperature, trickCallback)

    # trick objects    
    SM = StateMachine()
    xboxarm = Arm_XBOX()
    nod_trick = Nod()
    shake_trick = Shake()

    # start up test
    nod_trick.nod()
    rospy.sleep(4.0)
    shake_trick.shake() 
    rospy.sleep(2.0)

    # execute automaton
    rate = rospy.Rate(60)  # set rate to 60 hz
    while not rospy.is_shutdown():

        if sub > 0.68 and sub < 0.70:
            rospy.loginfo('if statement')
            nod_trick.nod()

        rate.sleep()