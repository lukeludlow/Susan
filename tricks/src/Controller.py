import numpy as np
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