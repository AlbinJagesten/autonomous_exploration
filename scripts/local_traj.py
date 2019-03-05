#!/usr/bin/env python

##Adapted from trajectory planner.py
import rospy
import numpy as np
import tf
import operator

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, Pose


class LocalPlanner:


    def __init__(self):
        """
        Initilizes trajectory planner. Subscribes to costmap and publishes a trajectory based on subscribed path message but
        with trimmed poses to take care of dangerous cells.
        """

        rospy.init_node('local_planner', anonymous=True)
        self.pose_new_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.trans_listener = tf.TransformListener()
        rospy.Subscriber('/costmap_2d', OccupancyGrid, self.costmap_callback)
        rospy.Subscriber('/trajectory', Path, self.traj_callback)
        


    def costmap_callback(self, msg):
        """
        Callback function for /costmap_2d topic.
        """
        
        self.metadata = msg.info
        self.w = self.metadata.width
        self.h = self.metadata.height      
        self.costmap = np.array(msg.data).reshape((self.h,self.w))
        self.fix_offset()


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


    def traj_callback(self,msg):
        """
        Takes the trajectory path and loops though each pose and publishes it till a pose is found that is in a "dangerous" cell 
        Once found, publishes the last feasible pose to the cmd_pose topic.

        PLEASE CORRECT FOR OFFSET IF NEEDED 
        """

        (translation,rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        path_msg=[]
        path_msg = msg.poses
        for i in range(len(path_msg)):
            x_pose = path_msg[i].pose.position.x
            y_pose = path_msg[i].pose.position.y
            cell_x = int(np.floor(x_pose / self.metadata.resolution) + self.w / 2) - self.x_offset
            cell_y = int(np.floor(y_pose / self.metadata.resolution) + self.h / 2) - self.y_offset
            if(self.costmap[cell_y,cell_x]>=99):
                if(i==0):
                    x = translation[0]
                    y = translation[1]
                    quaternion = (
                                rotation.x,
                                rotation.y,
                                rotation.z,
                                rotation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    self.theta = euler[2]
                    pose_g_msg = Pose2D()
                    pose_g_msg.x = x
                    pose_g_msg.y = y
                    pose_g_msg.theta = self.theta
                    self.pose_new_goal_publisher.publish(pose_g_msg)
                    return
                return
            
            else:
                # quaternion = (
                #             path_msg[i].pose.orientation.x,
                #             path_msg[i].pose.orientation.y,
                #             path_msg[i].pose.orientation.z,
                #             path_msg[i].pose.orientation.w)
                # euler = tf.transformations.euler_from_quaternion(quaternion)
                pose_g_msg = Pose2D()
                pose_g_msg.x = x_pose
                pose_g_msg.y = y_pose
                pose_g_msg.theta = np.arctan2((y_pose - translation[1]),(x_pose - translation[0]))
                self.pose_new_goal_publisher.publish(pose_g_msg)
        return


    def run(self):
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":

    local_traj_plan = LocalPlanner()
    local_traj_plan.run()
    
    
