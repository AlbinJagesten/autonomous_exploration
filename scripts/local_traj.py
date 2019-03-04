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
        Initilizes trajectory planner. Subscribes to costmap and publishes a trajectory to the closest unknown cell.
        """

        rospy.init_node('local_planner', anonymous=True)
        rospy.Subscriber('/costmap_2d', OccupancyGrid, self.costmap_callback)
        rospy.Subscriber('/trajectory', Path, self.traj_callback)
        self.pose_new_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.trans_listener = tf.TransformListener()
        


    def costmap_callback(self, msg):
        """
        Callback function for /costmap_2d topic.
        """
        
        self.metadata = msg.info
        self.w = self.metadata.width
        self.h = self.metadata.height      
        self.costmap = np.array(msg.data).reshape((self.h,self.w))



    def traj_callback(self,msg):
        """
        Takes the trajectory path and loops though each pose till a pose is found that is in a "dangerous" cell 
        Once found, publishes the last feasible pose to the cmd_pose topic.

        PLEASE CORRECT FOR OFFSET IF NEEDED 
        """

        (translation,rotation) = self.trans_listener.lookupTransform("/map", "/base_footprint", rospy.Time(0))
        path_msg=[]
        path_msg = msg.poses
        for i in range(len(path_msg)):
            x_pose = path_msg[i].pose.position.x
            y_pose = path_msg[i].pose.position.y
            cell_x = int(np.floor(x_pose / self.metadata.resolution) + self.w / 2)
            cell_y = int(np.floor(y_pose / self.metadata.resolution) + self.h / 2)
            if(self.costmap[cell_y,cell_x]>90):
                if(i==0):
                    self.x = translation[0]
                    self.y = translation[1]
                    quaternion = (
                                path_msg[i].pose.orientation.x,
                                path_msg[i].pose.orientation.y,
                                path_msg[i].pose.orientation.z,
                                path_msg[i].pose.orientation.w)
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    self.theta = euler[2]
                    pose_g_msg = Pose2D()
                    pose_g_msg.x = self.x
                    pose_g_msg.y = self.y
                    pose_g_msg.theta = self.theta
                    self.pose_new_goal_publisher.publish(pose_g_msg)
                    return
                quaternion = (
                            path_msg[i].pose.orientation.x,
                            path_msg[i].pose.orientation.y,
                            path_msg[i].pose.orientation.z,
                            path_msg[i].pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                pose_g_msg = Pose2D()
                pose_g_msg.x = x_pose
                pose_g_msg.y = y_pose
                pose_g_msg.theta = euler[2]
                self.pose_new_goal_publisher.publish(pose_g_msg)
                return
        x_pose = path_msg[-1].pose.position.x
        y_pose = path_msg[-1].pose.position.y
        quaternion = (
                    path_msg[-1].pose.orientation.x,
                    path_msg[-1].pose.orientation.y,
                    path_msg[-1].pose.orientation.z,
                    path_msg[-1].pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # euler = tf.transformations.euler_from_quaternion(path_msg[-1].pose.orientation)
        pose_g_msg = Pose2D()
        pose_g_msg.x = x_pose
        pose_g_msg.y = y_pose
        pose_g_msg.theta = euler[2]
        self.pose_new_goal_publisher.publish(pose_g_msg)
        return           


    def run(self):
        while not rospy.is_shutdown():
            pass


if __name__ == "__main__":

    local_traj_plan = LocalPlanner()
    local_traj_plan.run()
    
    
