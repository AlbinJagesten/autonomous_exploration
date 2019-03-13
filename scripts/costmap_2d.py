#!/usr/bin/env python

'''
costmap_2d: This node overlays the /map published by /gmapping
with costs as per a convention mentioned below, suitable for 
autonomous exploration and mapping.

AUTHORS: Albin Jagesten and Varun Nayak
'''

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

DECAY_RADIUS_MULTIPLIER = 10 # R = DECAY_RADIUS_MULTIPLIER*r
ROBOT_DIAMETER = 0.1 #15cm

class CostMap:
    """
    Subscribes to "/map" and creates a costmap by inflating walls by robot radius.
    """


    def __init__(self, radius):
        """
        Initializes CostMap node.
        """

        rospy.init_node('costmap_2d', anonymous=True)
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        self.pub = rospy.Publisher('costmap_2d', OccupancyGrid, queue_size=1)
        rospy.Subscriber('/exploration_complete', Bool, self.exploration_complete_callback)
        
        self.map = None
        self.metadata = None
        self.costmap = None
        self.w = 0
        self.h = 0
        self.radius = radius
        self.rate = rospy.Rate(1)


    def exploration_complete_callback(self, msg):
        """
        Terminates the node if exploration is complete.
        """

        exploration_complete = msg.data

        if exploration_complete:
            rospy.signal_shutdown("Exploration complete.")


    def map_callback(self, msg):
        """
        Callback function for /map topic. Updates the costmap according to the latest map.
        """
        self.metadata = msg.info
        self.w = self.metadata.width
        self.h = self.metadata.height
        self.map = np.array(msg.data).reshape((self.h,self.w))

        if self.costmap is None:
            self.costmap = np.zeros((self.h,self.w))
            for i in range(self.h/2 - 10, self.h/2 + 18):
                for j in range(self.w/2 - 10, self.w/2 + 18):
                    if self.costmap[i,j] == 0:
                        self.costmap[i,j] = 1

        self.update_costmap()
        

        print("max",np.max(self.costmap))
        print("min",np.min(self.costmap))


    def update_costmap(self):
        """
        Updates the costmap.

        Original /map values:
            100 - Obstacle
              0 - Free space 
             -1 - Unknown
            
        Cost map values:
            100 - Obstacle (lethal)
             99 - Inflation radius (lethal)
           98:2 - decay from obstacle
              1 - Free space
              0 - Unknown 
        """

        #print("Mapval",self.map[self.h/2,self.w/2])


        for y in range(self.h):
            for x in range(self.w):
                val = self.map[y,x]



                if val == 100 and self.costmap[y,x] !=100: #obstacle
                    self.costmap[y,x] = 100
                    self.make_square(y,x)
                    self.decay_obstacle(y,x)

                elif val == 0 and self.costmap[y,x] ==0: #known free space
                    self.costmap[y,x] = 1



    def make_square(self,y,x):
        """
        Makes an inflated square of size (2r x 2r) around obstacle point (x,y).
        """
        
        r = int(np.ceil(self.radius / self.metadata.resolution))
        min_x = max(0, x - r)
        max_x = min(self.w , x + r + 1)
        min_y = max(0, y - r)
        max_y = min(self.h , y + r + 1)

        for i in range(min_y,max_y):
            for j in range(min_x,max_x):
                if self.costmap[i,j] != 100:
                    self.costmap[i,j] = 99

    def decay_obstacle(self,y,x):
    	"""
    	Adds a cost as per a decay function around an obstacle
    	"""

    	r = int(np.ceil(self.radius / self.metadata.resolution))
    	R = DECAY_RADIUS_MULTIPLIER*r 	#add decay for distance 5*r 
        min_x = max(0, x - R)
        max_x = min(self.w , x + R + 1)
        min_y = max(0, y - R)
        max_y = min(self.h , y + R + 1)

        
        for i in range(min_y,max_y):
            for j in range(min_x,max_x):

            	#calculate the decay value:
            	dist_x = np.absolute(x - j)
            	dist_y = np.absolute(y - i)
            	dist = np.sqrt(dist_x**2 + dist_y**2)

            	#implementing linear cost decay
            	cost_decay = (dist - R)*(98-2)/(r-R) + 2

            	cost_decay_int = np.rint(cost_decay)


                if (self.costmap[i,j] < cost_decay_int) and (cost_decay_int < 99) and (cost_decay_int > 1):
                    self.costmap[i,j] = cost_decay_int
        

    """
    def update_unknown_space(self,y,x):

    	if self.map[y,x] == -1 and self.costmap[y,x] == 1:	#if unknown and was previously free space (non obstacle)
    		self.costmap[y,x] = 0
    """



    def run(self):
        """
        Starts ros loop
        """

        while not rospy.is_shutdown():
            if self.costmap is not None:
                costmap_msgs = OccupancyGrid()
                costmap_msgs.info = self.metadata
                costmap_msgs.data = np.ravel(self.costmap).tolist()
                self.pub.publish(costmap_msgs)
            self.rate.sleep()


if __name__ == "__main__":

    costmap = CostMap(ROBOT_DIAMETER/ 2.0)
    costmap.run()
