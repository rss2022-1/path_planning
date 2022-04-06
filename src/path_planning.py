#!/usr/bin/env python

from sklearn.metrics import jaccard_score
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
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
        
        self.goal = Point()
        self.start = Point()
        self.map = None
        self.map_resolution = 1.0
        self.map_origin_pos = np.zeros((3, 1)) # [x, y, z]
        self.map_origin_rot = np.zeros((4, 1)) # [x, y, z, w]
        self.occupancy_threshold = 50

        self.search = True # True for search-based planning, False for sample-based planning

    def map_cb(self, msg):
        # convert map data into a 2D numpy array indexed by (u,v) where 
        # self.map[v][u] = 1 means the cell at (u, v) is occupied, while
        # = 0 means it is not occupied
        self.map = np.reshape(msg.data, (msg.info.height, msg.info.width))
        self.map = np.where(self.map < 0, 1, self.map)
        self.map = np.where(self.map > self.occupancy_threshold, 1, 0)
        self.map_resolution = msg.info.resolution
        self.map_origin_pos[0] = msg.info.origin.position.x
        self.map_origin_pos[1] = msg.info.origin.position.y
        self.map_origin_pos[2] = msg.info.origin.position.z
        self.map_origin_rot[0] = msg.info.origin.orientation.x
        self.map_origin_rot[1] = msg.info.origin.orientation.y
        self.map_origin_rot[2] = msg.info.origin.orientation.z
        self.map_origin_rot[3] = msg.info.origin.orientation.w


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

        Pose.msg
            Point position
            Quaternion orientation

        Point.msg
            float64 x
            float64 y
            float64 z
        """
        start_xy = msg.pose.pose.position
        self.start = self.convert_xy_to_uv(start_xy)


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
        goal_xy = msg.pose.pose.position
        self.goal = self.convert_xy_to_uv(goal_xy)


    def get_rot_matrix_from_quaternion(self, q):
        """
        q = a 4x1 column vector representing a quaternion [x, y, z, w]
        """
        q0 = q[0]
        q1 = q[1]
        q2 = q[2]
        q3 = q[3]
        
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])

        return rot_matrix

    
    def get_transformation_matrix(self, rot_matrix, transl_matrix):
        """
        Given rotation matrix R, and translation matrix t, returns transformation matrix T:
        T = | R t |
            | 0 1 |
        """
        homogenous_row = np.array([[0, 0, 0, 1]])
        rot_and_transl = np.hstack(rot_matrix, transl_matrix)
        transform_matrix = np.vstack(rot_and_transl, homogenous_row)
        return transform_matrix


    def convert_xy_to_uv(self, point):
        """
        point is a Point in the xy-coordinate system
        returns a Point in uv-coordinate system
        """
        new_pose = np.zeros(4, 1)
        new_pose[0] = point.x
        new_pose[1] = point.y
        new_pose[3] = 1
        translation_matrix = self.map_origin_pos
        rotation_matrix = self.get_rot_matrix_from_quaternion(self.map_origin_rot)
        transform_matrix = self.get_transformation_matrix(rotation_matrix, translation_matrix)
        transform_inv = np.linalg.inv(transform_matrix)
        uv_pose = transform_inv @ new_pose
        uv_pose = uv_pose / self.map_resolution
        uv_point = Point()
        uv_point.x = uv_pose[0]
        uv_point.y = uv_pose[1]
        uv_point.z = 0
        return uv_point

    def convert_uv_to_xy(self, point):
        """
        point is a Point in the uv-coordinate system
        returns a Point in xy-coordinate system
        """
        new_pose = np.zeros(4, 1)
        new_pose[0] = point.x
        new_pose[1] = point.y
        new_pose[3] = 1
        new_pose = self.map_resolution * new_pose
        translation_matrix = self.map_origin_pos
        rotation_matrix = self.get_rot_matrix_from_quaternion(self.map_origin_rot)
        transform_matrix = self.get_transformation_matrix(rotation_matrix, translation_matrix)
        xy_pose = transform_matrix @ new_pose
        xy_point = Point()
        xy_point.x = xy_pose[0]
        xy_point.y = xy_pose[1]
        xy_point.z = 0
        return xy_point


    def get_euclidean_distance(self, start_point, end_point):
        """
        Calculates the Euclidean distance between two points.
        Intended for use as a heuristic.
        Should work independent of coordinate frames.
        
        Inputs:
            start_point: position array
            end_point: position array

        Outputs:
            distance (float)
        """

        if len(start_point) != len(end_point):
            rospy.loginfo("Unable to compute Euclidean distance.")
            return

        return np.sqrt(np.sum(np.square(end_point-start_point)))

    def get_neighbors(self, point):
        """
        Finds all viable neighbors to a given point.
        If there is an obstacle at that location, do not include.
        Assumes 2D coordinates.
        Includes diagonals. 
        """
        plus = [-1, 0, 1]

        neighbors = {point} 

        for i in plus:  
            a = point[0] + i
            for j in plus:
                b = point[1] + j
                if self.map[b][a] == 0: # no obstacles
                    # assumes (u, v) coordinates
                    neighbors.add([a, b])

        return neighbors

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
        """
        currently assume start_point, end_point are arrays
        should they be points instead?
        """
        ## CODE FOR PATH PLANNING ##

        if abs(start_point.x - end_point.x) < 0.001 and abs(start_point.y - end_point.y) < 0.001:
            # if for some reason the start and end point are the same (1 mm tolerance),
            # then do nothing
            return

        if not self.trajectory.empty():
            self.trajectory.clear()

        if self.search: 
            path = self.astar_search(start_point, end_point, map)
        else:
            path = self.random_sampling_search(start_point, end_point, map)
        
        # path will be a list of points in (u, v) coordinates
        # transform to (x, y) coordinates
        # update self.trajectory
        # profit 

        for point in path:
            new_point = self.convert_uv_to_xy(point)
            self.trajectory.addPoint(new_point)

        ## ##

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
