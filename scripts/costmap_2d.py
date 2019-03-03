#!/usr/bin/env python

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid

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
        self.map = None
        self.metadata = None
        self.costmap = None
        self.w = 0
        self.h = 0
        self.radius = radius
        self.rate = rospy.Rate(1)


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
        self.update_costmap()
        


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
              1 - Free space
              0 - Unknown 
        """

        for y in range(self.h):
            for x in range(self.w):
                val = self.map[y,x]

                if val == 100 and self.costmap[y,x] != 100:
                    self.costmap[y,x] = 100
                    self.make_square(y,x)

                elif val == 0 and self.costmap[y,x] == 0:
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


    def run(self):
        """
        Starts ros loop
        """

        while not rospy.is_shutdown():
            if self.costmap is not None:
                costmap_msgs = OccupancyGrid()
                costmap_msgs.info = self.metadata
                costmap_msgs.info.origin.position.x = -(self.w / 2.0) * self.metadata.resolution
                costmap_msgs.info.origin.position.y = -(self.h / 2.0) * self.metadata.resolution
                costmap_msgs.data = np.ravel(self.costmap).tolist()
                self.pub.publish(costmap_msgs)
            self.rate.sleep()


if __name__ == "__main__":
    costmap = CostMap(0.15 / 2.0)
    costmap.run()