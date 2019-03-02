#!/usr/bin/env python

import rospy
import numpy as np
import tf
import operator

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose

class TrajectoryPlanner:


    def __init__(self):
        rospy.init_node('trajectory_planner', anonymous=True)
        rospy.Subscriber('/costmap_2d', OccupancyGrid, self.costmap_callback)
        self.trans_listener = tf.TransformListener()
        self.traj_pub = rospy.Publisher('/trajectory', Path, queue_size=1)


    def costmap_callback(self, msg):
        """
        Callback function for /costmap_2d topic
        """
        self.metadata = msg.info
        self.w = self.metadata.width
        self.h = self.metadata.height
        self.costmap = np.array(msg.data).reshape((self.h,self.w))
        self.find_trajectory()


    def find_trajectory(self, cm, cell_x, cell_y):

        """
        Return a trajectory i.e. a list of map coordinates
        """

        self.costmap = cm
        self.w = 6
        self.h = 6


        #translation,_ = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        #self.x = translation[0]
        #self.y = translation[1]
        
        #cell_x = int(np.round(self.x / self.metadata.resolution) + self.w / 2)
        #cell_y = int(np.round(self.y / self.metadata.resolution) + self.h / 2)

        
        
        #TODO: Add Dijkstra Algorithm to find Unkown location in costmap

        visited = np.zeros(self.costmap.shape)
        visited[cell_y,cell_x] = 1

        to_explore = self.add_neighbors(visited, Node(cell_x,cell_y,0,None))
        to_explore.sort(key=operator.attrgetter('cost'))

        while to_explore:   
            next_node = to_explore.pop(0)
            if next_node.cost == -1:
                self.get_trajectory(next_node)
                break
            
            to_explore = to_explore + self.add_neighbors(visited, next_node)
            to_explore.sort(key=operator.attrgetter('cost'))



    def get_trajectory(self, node):

    
        path_msg = Path()
        path_msg.poses = []
        path_msg.header.frame_id = "/map"
        
        while node is not None:

            point = PoseStamped()
            point.header.frame_id = "/map"
            point.pose.position.x = (node.x - self.w / 2)#*self.metadata.resolution
            point.pose.position.y = (node.y - self.h / 2)#*self.metadata.resolution

            print(point.pose.position.x)
            print(point.pose.position.y)
            

            path_msg.poses.append(point)

            node = node.parent


        current_point = PoseStamped()
        current_point.pose.position.x = -3
        current_point.pose.position.y = -2

        path_msg.poses.append(current_point)

        path_msg.poses.reverse()

        self.traj_pub.publish(path_msg)
            

    def add_neighbors(self,visited, parent):
        x = parent.x
        y = parent.y
        cost = parent.cost
        neighbors = []
        neighbor_grid = [(-1,1), (0,1), (1,1), (-1,0), (1,0), (-1,-1), (0,-1), (1,-1)]

        for idx in neighbor_grid:
            new_x = x + idx[0]
            new_y = y + idx[1]
            if self.valid_pos(new_x, new_y, visited):
                visited[new_y, new_x] = 1
                neighbors.append(self.new_node(new_x, new_y, cost, parent))

        return neighbors


    def valid_pos(self, x, y, visited):
        try:
            return visited[y,x] == 0 and self.costmap[y,x]<99
        except IndexError:
            return False


    def new_node(self, x, y, cost, parent):
        if self.costmap[y,x] == 0:
            return Node(x,y,-1,parent)
        return Node(x,y,cost+1,parent)


    def run(self):
        while not rospy.is_shutdown():
            pass


class Node:

    def __init__(self, x,y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent


if __name__ == "__main__":

    test_map = np.array([[100, 100, 1, 1, 0, 1],
                         [1, 100, 100, 1, 1, 1],
                         [1, 1, 100, 100, 1, 1],
                         [1, 1, 1, 100, 1, 1],
                         [1, 1, 1, 1, 1, 1],
                         [1, 1, 1, 1, 1, 1]])
    
    traj_plan = TrajectoryPlanner()
    traj_plan.find_trajectory(test_map, 0, 1)
    #traj_plan.run()
    
    
