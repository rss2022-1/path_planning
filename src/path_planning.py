#!/usr/bin/env python

# from sklearn.metrics import jaccard_score # what is this
from cgi import test
from cv2 import MARKER_TRIANGLE_DOWN
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory

import cv2 

class PathPlan(object):
    """ 
    Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        # self.odom_topic = rospy.get_param("~odom_topic")
        self.odom_topic = "/odom" # for simulation testing
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

        self.clicked_point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb)
        
        self.goal = Point()
        self.start = Point()
        self.map = None
        self.map_resolution = 1.0
        self.map_origin_pos = np.zeros((3, 1)) # [x, y, z]
        self.map_origin_rot = np.zeros((4, 1)) # [x, y, z, w]
        self.occupancy_threshold = 50

        self.search = True # True for search-based planning, False for sample-based planning

        self.map_dimensions = None


    def map_cb(self, msg):
        """
        Converts map data into a 2D numpy array indexed by (u, v) where 
        self.map[v][u] = 1 means the cell at (u, v) is occupied, while
        self.map[v][u] = 0 means it is not occupied
        """
        self.map = np.reshape(msg.data, (msg.info.height, msg.info.width)) # sanity check these values
        # self.map = np.where(self.map < 0, 1, self.map)
        # self.map = np.where(self.map > self.occupancy_threshold, 1, 0)

        self.map = np.where(self.map < 0, 1, self.map)
        # self.map = np.where(self.map > self.occupancy_threshold, self.map, 1)
        
        self.map_resolution = msg.info.resolution # CHECK THIS
        self.map_origin_pos[0] = msg.info.origin.position.x
        self.map_origin_pos[1] = msg.info.origin.position.y
        self.map_origin_pos[2] = msg.info.origin.position.z
        self.map_origin_rot[0] = msg.info.origin.orientation.x
        self.map_origin_rot[1] = msg.info.origin.orientation.y
        self.map_origin_rot[2] = msg.info.origin.orientation.z
        self.map_origin_rot[3] = msg.info.origin.orientation.w

        self.map_dimensions = (msg.info.height, msg.info.width)

        self.map = self.map.astype(np.uint8)
        self.map = cv2.dilate(self.map, np.ones((15, 15), 'uint8'), iterations=1)


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
        goal_xy = msg.pose.position
        self.goal = self.convert_xy_to_uv(goal_xy)


    def clicked_point_cb(self, msg):
        """
        Prints point in xy coordinates, then converts to uv coordinates.
        For debugging purposes.

        PointStamped.msg
            Header header
            Point point
        """
        rospy.loginfo("Clicked point (xy): (%d, %d)", msg.point.x, msg.point.y)
        uv_point = self.convert_xy_to_uv(msg.point)
        rospy.loginfo("Clicked point (uv): (%d, %d)", uv_point.x, uv_point.y)
        xy_point = self.convert_uv_to_xy(uv_point)
        rospy.loginfo("Occupation: %d", self.map[uv_point.y][uv_point.x])
        rospy.loginfo("----------------------------")


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
        rot_and_transl = np.hstack((rot_matrix[:,0], rot_matrix[:,1], rot_matrix[:,2], transl_matrix)) # changed this
        transform_matrix = np.vstack((rot_and_transl, homogenous_row))
        return transform_matrix


    def convert_xy_to_uv(self, point):
        """
        point is a Point in the xy-coordinate system
        returns a Point in uv-coordinate system

        convert to integer so can (ideally) index into self.map
        """
        new_pose = np.zeros((4, 1))
        new_pose[0] = point.x
        new_pose[1] = point.y
        new_pose[3] = 1
        translation_matrix = self.map_origin_pos
        rotation_matrix = self.get_rot_matrix_from_quaternion(self.map_origin_rot)
        transform_matrix = self.get_transformation_matrix(rotation_matrix, translation_matrix)
        transform_inv = np.linalg.inv(transform_matrix)
        uv_pose = np.dot(transform_inv, new_pose) # previously used @?
        uv_pose = uv_pose / self.map_resolution
        uv_point = Point()
        uv_point.x = int(uv_pose[0]) # convert to int
        uv_point.y = int(uv_pose[1]) * -1 # because pixel coordinates
        uv_point.z = 0
        return uv_point


    def convert_uv_to_xy(self, point):
        """
        point is a Point in the uv-coordinate system
        returns a Point in xy-coordinate system
        """
        new_pose = np.zeros((4, 1))
        new_pose[0] = point.x
        new_pose[1] = point.y * -1 # because pixel coordinates
        new_pose = self.map_resolution * new_pose
        new_pose[3] = 1
        translation_matrix = self.map_origin_pos
        rotation_matrix = self.get_rot_matrix_from_quaternion(self.map_origin_rot)
        transform_matrix = self.get_transformation_matrix(rotation_matrix, translation_matrix)
        xy_pose = np.dot(transform_matrix, new_pose) # previously used @?
        xy_point = Point()
        xy_point.x = float(xy_pose[0]) # convert to float, won't be used for indexing
        xy_point.y = float(xy_pose[1]) 
        xy_point.z = 0
        return xy_point


    def get_euclidean_distance(self, start_point, end_point):
        """
        Calculates the Euclidean distance between two points.
        Intended for use as a heuristic.
        Should work independent of coordinate frames.
        Assumes 2D points.
        
        Inputs:
            start_point: tuple
            end_point: tuple

        Outputs:
            distance (float)
        """
        if isinstance(start_point, tuple):
            vector = np.array([end_point[0]-start_point[0], end_point[1]-start_point[1]])

        return np.linalg.norm(vector)


    def get_neighbors(self, point):
        """
        Finds all neighbors to a given point.
        Can exclude points with obstacles.
        Assumes 2D coordinates.
        No diagonals. 

        Inputs: 
            point: tuple in u, v coordinates
            map_look: check if obstacle is at neighbor point

        Outputs:
            set of tuples
        """

        neighbors = set()
        
        for i in [-1, 1]:
            if point[0] + i <= self.map_dimensions[1] and point[0] + i >=0:
                new_point = (point[0] + i, point[1])
                if self.map[new_point[1]][new_point[0]] == 0:
                    neighbors.add(new_point)
        for i in [-1, 1]:
            if point[1] +i <= self.map_dimensions[0] and point[1] + i >=0:
                new_point = (point[0], point[1] + i)
                if self.map[new_point[1]][new_point[0]] == 0:
                    neighbors.add(new_point)
        
        return neighbors
    
    
    def make_new_point(self, x, y, z=0):
        """
        Given x, y, z (z opt.) coordinates, returns a point object.
        """
        new_point = Point()
        new_point.x = x
        new_point.y = y
        new_point.z = z

        return new_point


    def point_to_coords(self, point):
        """
        Input: 
            point: Point
        Output:
            tuple with (x, y) point coordinates
        """

        return (point.x, point.y)


    def bfs_search(self, start_point, end_point, map):
        """
        Written as a sanity check.
        Don't actually use this.
        """
        queue = []
        queue.append([start_point])

        while queue:
            path = queue.pop(0)
            node = path[-1]
            if node == end_point:
                return path
            else:
                for neighbor in self.get_neighbors(node):
                    new_path = path[:]
                    if neighbor not in path:
                        new_path.append(neighbor)
                        queue.append(new_path)


    def astar_search(self, start_point, end_point, map):
        """
        A* Search
        Sorts queue based on heuristic that is a sum of:
            distance from last point to end point, and
            total distance traveled so far

        Inputs:
            start_point: Point
            end_point: Point
            map: nd numpy array (from OccupancyGrid)

        Output:
            list of Points
        """
        queue = []
        # length 3 tuple of (heuristic, length of path, list of points in path)
        queue.append((self.get_euclidean_distance(start_point, end_point), 0, [start_point]))
        seen_points = {start_point}

        count = 0
    
        while queue:
            queue.sort(key=lambda k: k[0]) # sorts queue by heuristic, which is first element of tuples

            tup = queue.pop(0)
            path = tup[-1]
            node = path[-1]
            if node == end_point:
                print('Queued ' + str(count) + ' times.')
                return path
            else:
                for neighbor in self.get_neighbors(node):
                    if neighbor not in seen_points:
                        # if we haven't been there and the spot is not occupied...
                        new_path = path[:]
                        new_len = tup[1] + 1
                        new_heur = self.get_euclidean_distance(neighbor, end_point) + new_len
                        new_path.append(neighbor)
                        new_tup = (new_heur, new_len, new_path) # using length as a heuristic instead
                        seen_points.add(neighbor)
                        queue.append(new_tup)

            count += 1

        print('Queued ' + str(count) + ' times.')
        print('Failed to find path.')


    def random_sampling_search(self, start_point, end_point, map):
        # invoked if 1) A* too slow or 2) we gun for extra credit or 3) both
        pass


    def plan_path(self, start_point, end_point, map):
        """
        Plans a path from the start point to the end point.

        Inputs:
            start_point: Point
            end_point: Point
            map: nd numpy array (from OccupancyGrid)

        Output:
            returns nothing
            updates self.trajectory
            publishes self.trajectory
            visualizes self.trajectory

        """
        ## CODE FOR PATH PLANNING ##

        if start_point.x == end_point.x and start_point.y == end_point.y:
            # if for some reason the start and end point are the same
            # then do nothing
            return

        if not self.trajectory.empty():
            self.trajectory.clear()

        if self.search: 
            print('starting A* search')
            path = self.astar_search(self.point_to_coords(start_point), self.point_to_coords(end_point), map)
        else:
            path = self.random_sampling_search(start_point, end_point, map)
        
        print("Adding points to trajectory")
        for point in path:
            uv_point = self.make_new_point(point[0], point[1])
            new_point = self.convert_uv_to_xy(uv_point)
            self.trajectory.addPoint(new_point)

        ## ##
        
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


    def test_coordinate_conversions(self):
        """
        Initializes random points in uv coordinates,
        converts to xy,
        then converts back to uv.
        """
        map_shape = np.shape(self.map)
        uv_point = self.make_new_point(np.random.randint(0, map_shape[0]), np.random.randint(0, map_shape[1]))
        xy_point = self.convert_uv_to_xy(uv_point)
        uv_point_back_calculated = self.convert_xy_to_uv(xy_point)
        print(uv_point)
        print(xy_point)
        print(uv_point_back_calculated)
        assert uv_point.x == uv_point_back_calculated.x, "x should be %d, got %d" % (uv_point.x, uv_point_back_calculated.x)
        assert uv_point.y == uv_point_back_calculated.y, "y should be %d, got %d" % (uv_point.y, uv_point_back_calculated.y)
        print("test_coordinate_conversions..........OK!")    

    
    def test_get_neighbors(self):
        test_point = self.make_new_point(1, 1)
        neighbors = []
        for n in self.get_neighbors(test_point):
            neighbors.append(self.point_to_coords(n))

        neighbors.sort()

        known_neighbors = {self.make_new_point(0, 0), self.make_new_point(1, 0), self.make_new_point(2, 0),
                        self.make_new_point(0, 1), self.make_new_point(1, 1), self.make_new_point(2, 1), 
                        self.make_new_point(0, 2), self.make_new_point(1, 2), self.make_new_point(2, 2)}

        known_neighbors_list = []
        for k in known_neighbors:
            known_neighbors_list.append(self.point_to_coords(k))
        
        known_neighbors_list.sort()

        assert known_neighbors_list == neighbors, "neighbors are not the same"
        print("test_get_neighbors...................OK!")


    def test_get_distance(self):
        test_point = self.make_new_point(3, 4)
        test_point_2 = self.make_new_point(6, 4)
        test_point_3 = self.make_new_point(-2, 4)
        distance = self.get_euclidean_distance(test_point, test_point_2)
        assert distance == 3, "distance should be 3, got %d" % distance
        distance_2 = self.get_euclidean_distance(test_point, test_point_3)
        assert distance_2 == 5, "distance should be 5, got %d" % distance_2
        distance_3 = self.get_euclidean_distance(self.make_new_point(0, 0), self.make_new_point(1,1))
        assert distance_3 == np.sqrt(2), "distance should be sqrt(2), got %d" % distance_3
        p4 = self.make_new_point(513, 962)
        p5 = self.make_new_point(489, 960)
        d4 = self.get_euclidean_distance(p4, p5)
        assert d4 == np.sqrt((960-962)**2+(489-513)**2), "distance incorrect"

        print("test_get_distance....................OK!")


    def test_bfs_search(self):
        print("Trying BFS search")
        path = self.bfs_search(self.make_new_point(0,0), self.make_new_point(1,5), self.map)
        print(path)


    def test_astar_search(self):
        print("Testing A* search")
        start_time = time.time()
        # path = self.astar_search(self.make_new_point(513,962), self.make_new_point(489,959), self.map)
        path = self.astar_search((513,962), (489,959), self.map)
        end_time = time.time()
        print('It took ' + str(end_time-start_time) + ' seconds to find this path.')
    

    def test_plan_path_real(self):
        """
        """
        print("Testing path planning")
        print("Starting at (" + str(self.start.x) + ", " + str(self.start.y) + ")")
        print("Going to (" + str(self.goal.x) + ", " + str(self.goal.y) + ")")
        assert self.start.x >= 0 and self.start.y >= 0, "start point not in uv, x: %d, y: %d" % (self.start.x, self.start.y)
        assert self.goal.x >= 0 and self.goal.y >= 0, "goal point not in uv, x: %d, y: %d" % (self.goal.x, self.goal.y)
        start_time = time.time()        
        self.plan_path(self.start, self.goal, self.map)
        end_time = time.time()
        print('It took ' + str(end_time-start_time) + ' seconds to find this path.')
        print("Is the path visible?")


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    print('waiting for map...')
    while pf.map_dimensions is None:
        pass
    # pf.test_coordinate_conversions()
    # pf.test_get_neighbors()
    # pf.test_get_distance()

    # pf.test_bfs_search()
    # pf.test_astar_search()

    print('waiting for goal...')
    while pf.goal.x == 0:
        pass
    pf.test_plan_path_real()

    exit()

    rospy.spin()
