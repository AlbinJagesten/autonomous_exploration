#!/usr/bin/env python

import rospy
import numpy as np
import tf

from nav_msgs.msg import OccupancyGrid

class TrajectoryPlanner:


    def __init__(self):
        rospy.init_node('trajectory_planner', anonymous=True)
        rospy.Subscriber('/costmap_2d', OccupancyGrid, self.costmap_callback)
        self.trans_listener = tf.TransformListener()


    def costmap_callback(self, msg):
        """
        Callback function for /costmap_2d topic
        """
        self.metadata = msg.info
        self.w = self.metadata.width
        self.h = self.metadata.height
        self.costmap = np.array(msg.data).reshape((self.h,self.w))
        self.find_trajectory()


    def find_trajectory(self):

        translation,_ = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        x = int(np.round(translation[0] / self.metadata.resolution) + self.w / 2)
        y = int(np.round(translation[1] / self.metadata.resolution) + self.h / 2)
        

    def run(self):
        while not rospy.is_shutdown():
            pass


    class Node:


        def __init__(self, x,y, parent, cost):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent = parent


        def get_cost(self):
            return self.cost


        def get_parent(self):
            return self.parent

        
        def get_pos(self):
            return self.x, self.y


if __name__ == "__main__":
    traj_plan = TrajectoryPlanner()
    #traj_plan.run()
    
    