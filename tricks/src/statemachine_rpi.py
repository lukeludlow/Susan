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
        rospy.init_node('state_machine')
        rospy.loginfo('state machine')
        self.rate = rospy.Rate(60)  # set rate to 60 hz
        # publishers


        #rospy.init_node('automaton')

        self.sub = rospy.Subscriber('/tricks', Temperature, self.trickCallback)


        self.nod_trick = Nod()
        self.shake_trick = Shake()



    def trickCallback(self, msg):
        rospy.loginfo('trickCallback')
        message = msg.average_temperature

        if message > 0.68 and message < 0.70:
            rospy.loginfo('callback if statement')
            self.nod_trick.nod()



        if message > 0.58 and message < 0.60:
            rospy.loginfo('callback if statement')
            self.shake_trick.shake()


        if message > 0.58 and message < 0.60:
            rospy.loginfo('callback if statement')
            self.shake_trick.dab()



















if __name__ == '__main__':

    
    rospy.loginfo('###')
    rospy.loginfo('### automaton running!')
    rospy.loginfo('###')

    # trick objects    
    SM = StateMachine()
    xboxarm = Arm_XBOX()
    #nod_trick = Nod()
    #shake_trick = Shake()

    # start up test
    #nod_trick.nod()
    #rospy.sleep(4.0)
    #shake_trick.shake() 
    #rospy.sleep(2.0)

    #SM.nod_trick.nod()

    # execute automaton
    rate = rospy.Rate(60)  # set rate to 60 hz
    while not rospy.is_shutdown():



        rate.sleep()