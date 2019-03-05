#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose2D

POINT_STEP = 5

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

class LocalPlanner:

    def __init__(self):

        rospy.init_node("local_planner", anonymous=True)

        rospy.Subscriber('/trajectory', Path, self.trajectory_callback)
        self.pose_pub = rospy.Publisher('/cmd_pose', Pose2D, queue_size=1) 
        
        
    def trajectory_callback(self, path_msg):

        #for i in range(0, len(path_msg.poses), POINT_STEP):
        i = min(POINT_STEP - 1, len(path_msg.poses)-2)
            
        current_x = path_msg.poses[i].pose.position.x
        current_y = path_msg.poses[i].pose.position.y
        next_x = path_msg.poses[i+1].pose.position.x
        next_y = path_msg.poses[i+1].pose.position.y
            
        pose = Pose2D()
        pose.x = current_x
        pose.y = current_y
        pose.theta = angle_between(np.array([current_x, current_y]), np.array([next_x, next_y]))
        self.pose_pub.publish(pose)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    lcl_planner = LocalPlanner()
    lcl_planner.run()
