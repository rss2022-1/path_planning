#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        
        self.goal = PoseStamped()
        self.start = Pose()
        self.map = None
        self.occupancy_threshold = 50

    def map_cb(self, msg):
        # convert map data into a 2D numpy array indexed by (u,v) where 
        # self.map[v][u] = 1 means the cell at (u, v) is occupied, while
        # = 0 means it is not occupied
        self.map = np.reshape(msg.data, (msg.info.height, msg.info.width))
        self.map = np.where(self.map < 0, 1, self.map)
        self.map = np.where(self.map > self.occupancy_threshold, 1, 0)


    def odom_cb(self, msg):
        self.start = msg.pose.pose


    def goal_cb(self, msg):
        self.goal = msg


    def convert_xy_to_uv(self, pose):
        # TODO: apply translation and rotation inverse from:
        # | R T |
        # | 0 1 |
        # divide by resolution

        # use to convert start and end positions into (u, v) frame for plan_path()
        pass 


    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
