#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

class PathPlan(object):
    """ 
    Listens for goal pose published by RViz and uses it to plan a path from
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

        self.search = True # True for search-based planning, False for sample-based planning

    def map_cb(self, msg):
        # convert map data into a 2D numpy array indexed by (u,v) where 
        # self.map[v][u] = 1 means the cell at (u, v) is occupied, while
        # = 0 means it is not occupied
        self.map = np.reshape(msg.data, (msg.info.height, msg.info.width))
        self.map = np.where(self.map < 0, 1, self.map)
        self.map = np.where(self.map > self.occupancy_threshold, 1, 0)


    def odom_cb(self, msg):
        """
        Odometry.msg
            Header header
            string child_frame_id
            geometry_msgs/PoseWithCovariance pose
            geometry_msgs/TwistWithCovariance twist

        PoseWithCovariance.msg
            Pose pose
            # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
            float64[36] covariance
        """
        self.start = msg.pose.pose[:3] # [x, y, z]


    def goal_cb(self, msg):
        """
        PoseStamped.msg
            Header header
            Pose pose

        Pose.msg
            Point position
            Quaternion orientation

        Point.msg
            float64 x
            float64 y
            float64 z
        """
        self.goal = [msg.pose.Point.x, msg.pose.Point.y, msg.pose.Point.z] # [x, y, z]


    def convert_xy_to_uv(self, pose):
        # TODO: apply translation and rotation inverse from:
        # | R T |
        # | 0 1 |
        # divide by resolution

        # use to convert start and end positions into (u, v) frame for plan_path()
        pass 

    def get_euclidean_distance(self, start_point, end_point):
        """
        Calculates the Euclidean distance between two points.
        Intended for use as a heuristic.
        
        Inputs:
            start_point: position array
            end_point: position array

        Outputs:
            distance (float)
        """

        if len(start_point) != len(end_point)
            rospy.loginfo("Unable to compute Euclidean distance.")
            return

        return np.sqrt(np.sum(np.square(end_point-start_point)))

    def get_neighbors(self, point):
        """
        Finds all viable neighbors to a given point.
        If there is an obstacle at that location, do not include.
        """
        pass

    def astar_search(self, start_point, end_point, map):
        queue = []
        # length 3 tuple of (distance to end, length of path, list of points in path)
        queue.append((self.get_euclidean_distance(start_point, end_point), 0, [start_point]))
        
        while queue:
            queue.sort() # sorts queue by heuristic
            tup = queue.pop(0)
            path = tup[-1]
            node = tup[-1][-1]
            if node == end_point:
                return path
            else:
                for neighbor in self.get_neighbors(node):
                    new_path = path.copy()
                    if neighbor not in path:
                        new_heur = self.get_euclidean_distance(neighbor, end_point)
                        new_len = tup[1] + self.get_euclidean_distance(node, neighbor)
                        new_path.append(neighbor)
                        queue.append((new_heur, new_len, new_path))

    def random_sampling_search(self, start_point, end_point, map):
        # invoked if 1) A* too slow or 2) we gun for extra credit or 3) both
        pass

    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##

        if start_point == end_point:
            # if for some reason the start and end point are the same,
            # then do nothing
            return

        if self.search: # I think this updates self.trajectory?
            self.astar_search(start_point, end_point, map)
        else:
            self.random_sampling_search(start_point, end_point, map)

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
