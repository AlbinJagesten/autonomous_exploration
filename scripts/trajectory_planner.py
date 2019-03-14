#!/usr/bin/env python

import rospy
import numpy as np
import tf
import operator

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Pose2D
from std_msgs.msg import Bool

# Threshold for considering an unknown cell a candidate goal cell
NUMBER_UNKNOWN_NEIGHBORS = 2

# Threshold for number of fails after which we terminate the search
NUMBER_OF_FAILS = 3

class TrajectoryPlanner:


    def __init__(self):
        """
        Initilizes trajectory planner. Subscribes to costmap and publishes a trajectory to the closest unknown cell.
        """
        
        rospy.init_node('trajectory_planner', anonymous=True)
        
        rospy.Subscriber('/costmap_2d', OccupancyGrid, self.costmap_callback)
        rospy.Subscriber('/exploration_complete', Bool, self.exploration_complete_callback)
        self.trans_listener = tf.TransformListener()
        
        self.traj_pub = rospy.Publisher('/cmd_path', Path, queue_size=1)
	self.auto_goal_pub = rospy.Publisher('/auto_goal', Pose2D, queue_size=1)
        self.exp_complete_pub = rospy.Publisher('/exploration_complete', Bool, queue_size=1)

        self.number_of_fails = 0
        

    def exploration_complete_callback(self, msg):
        """
        Terminates the node if exploration is complete.
        """

        exploration_complete = msg.data

        if exploration_complete:
            rospy.signal_shutdown("Exploration complete.")


    def costmap_callback(self, msg):
        """
        Callback function for /costmap_2d topic.
        """
        
        self.metadata = msg.info
        self.w = self.metadata.width
        self.h = self.metadata.height
        self.fix_offset()      
        self.costmap = np.array(msg.data).reshape((self.h,self.w))
        self.find_trajectory()


    def fix_offset(self):
        """
        Corrects map offset.
        """
        
        res = self.metadata.resolution
        
        x_diff = self.w / 2 * res + self.metadata.origin.position.x
        if x_diff < 0:
            self.x_offset = int(np.round(x_diff / res))
        else:
            self.x_offset = -int(np.round(x_diff / res))
            
        y_diff = self.h / 2 * res + self.metadata.origin.position.y
        if y_diff < 0:
            self.y_offset = int(np.round(y_diff / res))
        else:
            self.y_offset = -int(np.round(y_diff / res))


    def find_trajectory(self):
        """
        Return a trajectory i.e. a list of map coordinates.
        Runs a modified Dijkstra algorithm to find the closest unknown cell.
        """

        translation,_ = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        self.x = translation[0]
        self.y = translation[1]
        
        cell_x = int(np.floor(self.x / self.metadata.resolution) + self.w / 2) - self.x_offset
        cell_y = int(np.floor(self.y / self.metadata.resolution) + self.h / 2) - self.y_offset

        visited = np.zeros(self.costmap.shape)
        visited[cell_y,cell_x] = 1

        to_explore = self.add_neighbors(visited, Node(cell_x,cell_y,0,None))
        to_explore.sort(key=operator.attrgetter('cost'))

        # Run modified Dijkstra algorithm
        while to_explore:   
            next_node = to_explore.pop(0)
            if next_node.cost == -1:
                print("Found goal!")
		self.send_final_pose(next_node)
                self.number_of_fails = 0
                self.get_trajectory(next_node)
                return
            
            to_explore = to_explore + self.add_neighbors(visited, next_node)
            to_explore.sort(key=operator.attrgetter('cost'))

        self.number_of_fails += 1
        print("Failed: %d times % self.number_of_fails")

        if self.number_of_fails >= NUMBER_OF_FAILS:
            print("Exiting!")
            msg = Bool()
            msg.data = True
            self.exp_complete_pub.publish(msg)

    def add_neighbors(self, visited, parent):
        """
        Returns adjacent neighbors that are not an obstacle and not already visited.
        """
        
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
                if self.valid_cost(x,y):
                    new_cost = cost + np.linalg.norm(idx)*self.costmap[new_y, new_x]
                    neighbors.append(self.new_node(new_x, new_y, new_cost, parent))

        return neighbors


    def valid_pos(self, x, y, visited):
        if x > -1 and y > -1 and x < self.w and y < self.h:
            return visited[y,x] == 0
        return False

    def valid_cost(self, x, y):
        return self.costmap[y,x]<99


    def new_node(self, x, y, cost, parent):
        if self.costmap[y,x] == 0:
            if self.count_unknown_neighbors(x, y) >= NUMBER_UNKNOWN_NEIGHBORS:
                return Node(x,y,-1,parent)
        return Node(x,y,cost,parent)

    def count_unknown_neighbors(self, x, y):

        neighbor_grid = [(-1,1), (0,1), (1,1), (-1,0), (1,0), (-1,-1), (0,-1), (1,-1)]
        
        count = 0
        for idx in neighbor_grid:
            new_x = x + idx[0]
            new_y = y + idx[1]
            if self.costmap[new_y, new_x] == 0:
                count += 1

        return count


    def send_final_pose(self, node):
        msg = Pose2D()
        msg.x = (node.x - self.w / 2 + 0.5 + self.x_offset)*self.metadata.resolution
            msg.y = (node.y - self.h / 2 + 0.5 + self.y_offset)*self.metadata.resolution
        dot = node.x * node.parent.x + node.y * node.parent.y
        det = node.parent.x * node.y - node.x * node.parent.y
        msg.theta = np.arctan2(det, dot)
        self.auto_goal_pub.publish(msg)


    def get_trajectory(self, node):

        path_msg = Path()
        path_msg.poses = []
        path_msg.header.frame_id = "/map"
        
        while node is not None:

            point = PoseStamped()
            point.header.frame_id = "/map"
            point.pose.position.x = (node.x - self.w / 2 + 0.5 + self.x_offset)*self.metadata.resolution
            point.pose.position.y = (node.y - self.h / 2 + 0.5 + self.y_offset)*self.metadata.resolution

            path_msg.poses.append(point)

            node = node.parent

        path_msg.poses.reverse()

        self.traj_pub.publish(path_msg)


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

    traj_plan = TrajectoryPlanner()
    traj_plan.run()
    
    
