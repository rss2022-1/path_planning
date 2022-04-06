#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped, Point32
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        # self.odom_topic       = rospy.get_param("~odom_topic")
        # self.lookahead        = rospy.get_param("~lookahead", .5)
        self.lookahead = 2
        self.speed            = 0
        self.wrap             = 0
        self.wheelbase_length = 0
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.localization_subscriber = rospy.Subscriber("/pf/pose/odom", Odometry, self.drive, queue_size = 1)
        self.current_pose = None
        self.prev_pose = None
        self.more_prev_pose = None

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

    def drive(self, msg):
        ''' Computes the steering angle and speed for the robot to follow the given
            trajectory.
        '''
        # FILL IN #
        # Pass back previous poses
        self.more_prev_pose = self.prev_pose
        self.prev_pose = self.current_pose
        # Get current pose
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w)
        # Find closest point on trajectory
        closest_point = self.find_closest_point_on_trajectory(self.current_pose)
        # Find lookahead point
        lookahead_point = self.find_lookahead_point(self.current_pose, closest_point)
        # Compute the steering angle and speed
        steering_angle = self.compute_steering_angle(lookahead_point)
        # Publish the drive command
        # Return the drive command

    def test_find_closest_point_on_trajectory(self):
        print("Testing find_closest_point_on_trajectory")
        current_pose = [0, 0, 0]
        self.trajectory.points = [[0, 1], [1, 1], [2, 20]]
        closest_index = self.find_closest_point_on_trajectory(current_pose)
        assert closest_index == 0, "Closest index should be 0, got %d" % closest_index
        print("test_find_closest_point_on_trajectory..........OK!")

    def find_closest_point_on_trajectory(self, current_pose):
        ''' Computes the closest point on the trajectory to the given pose. RETURN THE INDEX
        '''
        # FILL IN # ALEX
        # Compute the closest point on the trajectory to the given pose
        # https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment/1501725#1501725
        # Return the closest point and segment on the trajectory

        # SLOW VERSION
        # current_point = np.array([current_pose[0], current_pose[1]])
        # closest_points = []
        # for i in range(len(self.trajectory.points)-1):
        #     p1 = np.array(self.trajectory.points[i])
        #     p2 = np.array(self.trajectory.points[i+1])
        #     t = max(0, min(1, np.dot(current_point - p1, p2 - p1) / np.linalg.norm(p2 - p1)**2))
        #     closest_point = p1 + t * (p2 - p1)
        #     closest_points.append(closest_point)
        # closest_index = np.argmin(np.linalg.norm(np.array(closest_points) - current_point, axis=1))
        # return closest_index

        # FAST VERSION
        current_point = np.array([current_pose[0], current_pose[1]])
        p1_array = np.array(self.trajectory.points[0:-1])
        p2_array = np.array(self.trajectory.points[1:])
        t_array = np.dot(current_point - p1_array, p2_array - p1_array) / np.linalg.norm(p2_array - p1_array)**2
        t_array = np.clip(t_array, 0, 1)
        closest_points = p1_array + t_array * (p2_array - p1_array)
        closest_index = np.argmin(np.linalg.norm(closest_points - current_point, axis=1))
        return closest_index


    def find_lookahead_point(self, current_pose, start_point_idx):
        ''' Computes the lookahead point for the given trajectory. The lookahead point
            is the point on the trajectory that is closest to the intersection of the
            circle defined by the robot's current position and the lookahead radius.
        '''
        # FILL IN # JORDAN
        # Note: Only look at points further ahead on the trajectory than the
        # point returned by find_closest_point_on_trajectory
        points = self.trajectory.points[start_point_idx:]
        distances = self.trajectory.distances[start_point_idx:]
        center = current_pose[:-1]
        rospy.loginfo("get lookahead")

        # Compute the lookahead point
        # NOT IN A FUNCTION TO REDUCE OVERHEAD
        for i in range(len(distances)):
            p1 = points[i]
            p2 = points[i+1]
            V = p2 - p1
            a = V.dot(V)
            b = 2 * V.dot(p1-center)
            c = p1.dot(p1) + center.dot(center) - 2 * p1.dot(center) - self.lookahead**2
            disc = b**2 - 4 * a * c
            if disc < 0:
                continue
            else:
                sqrt_disc = np.sqrt(disc)
                t1 = (-b + sqrt_disc) / (2 * a)
                t2 = (-b - sqrt_disc) / (2 * a)
                if 0 <= t1 <= 1 and 0 <= t2 <= 1:
                    # choose which one
                    t = min(t1, t2)
                elif 0 <= t1 <= 1:
                    t = t1
                elif 0 <= t2 <= 1:
                    t = t2
                else:
                    continue
                res = p1 + t1 * V
                # self.publish_point(res)
                return res
        # Intersection not found, how to find point to go to?
        rospy.loginfo("COULD NOT FIND INTERSECTION DO SOMETHING")

        return min(start_point_idx + 2, len(distances))

    def test_compute_intersection(self):
        print("TESTING intersection point...")
        current_pose = np.array([0, 0, 0])
        self.trajectory.points = np.array([[0, 0], [1, 0], [2, 0], [3,0]])
        self.trajectory.distances = np.array([1,1,1])
        self.lookahead = 2
        int_point  = self.find_lookahead_point(current_pose, 0)
        print("point: ", int_point)
        assert int_point[0] == 2, "Closest point x should be 2, got %d" % int_point[0]
        print("test_find_lookahead_point..........OK!")
        print("TESTING intersection point 2....")
        current_pose = np.array([0, 0, 0])
        self.trajectory.points = np.array([[0, 0], [2, 0], [4, 0], [6,0]])
        self.trajectory.distances = np.array([2,2,2])
        self.lookahead = 3
        int_point  = self.find_lookahead_point(current_pose, 0)
        print("point: ", int_point)
        assert int_point[0] == 3, "Closest point x should be 3, got %d" % int_point[0]
        print("test_find_lookahead_point 2..........OK!")

    def publish_point(self, res):
        cloud = PointCloud()
        cloud.header.frame_id = "/map"
        cloud.points = [Point32()]
        cloud.points[0].x = res[0]
        cloud.points[0].y = res[1]
        rospy.loginfo("found point")
        self.intersection_pub.publish(cloud)



    def compute_steering_angle(self, lookahead_point):
        ''' Computes the steering angle for the robot to follow the given trajectory.
        '''
        # FILL IN # KYRI
        # Compute eta - use a dot b = |a|*|b|*cos(eta) where a is our forward velocity and
        # b is the vector from the robot to the lookahead point
        # Compute the steering angle
        if self.more_prev_pose is None:
            return 0
        else:
            x_curr, y_curr, _ = self.current_pose
            x_prev, y_prev, _ = self.more_prev_pose
            x_ref, y_ref = lookahead_point
            car_vector = (x_curr - x_prev, y_curr, y_prev) # direction of car
            reference_vector = (x_ref - x_curr, y_ref - y_curr) # car to reference point
            l_1 = np.linalg.norm(reference_vector)
            eta = np.arccos(np.dot(car_vector, reference_vector)/(np.linalg.norm(car_vector)*l_1))
            delta = np.arctan(2 * self.wheelbase_length * np.sin(eta) / l_1) # from lecture notes 5-6
            return delta

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    pf.test_find_closest_point_on_trajectory()
    pf.test_compute_intersection()
    rospy.spin()
