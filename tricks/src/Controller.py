import numpy as np
import tf.transformations as tr
from PID import PID
import rospy


class Controller:
    def __init__(self):
        self.name = 'default'
        self.default_status = 'Waiting for user input'
        self.status = self.default_status

        # Constants
        self.FINAL_GOAL_THRESH = 2  # m
        self.INTER_GOAL_THRESH = 4  # m

        # These variables are set by StateMachine
        self.desired_heading = [0,0]    # [rad, deg]
        self.current_heading = [0,0]    # [rad, deg]
        self.heading_offset = [0,0]     # [rad, deg]
        self.current_distance = 100  # (m)
        self.num_waypoints = 0

        # These are set in navigation/param/navigation_param.yaml
        self.W_MAX = rospy.get_param('~W_MAX') # Maximum angular velocity of rover [rad/s]
        self.DEFAULT_V = rospy.get_param('~V_MAX') # Default forward velocity of rover [m/s]