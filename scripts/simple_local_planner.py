#!/usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class LocalPlanner:

    def __init__(self):

        rospy.Subscriber('/trajectory', Path, self.trajectory_callback)

    def trajectory_callback(self, path_msg):
        # TODO
        
        pass
