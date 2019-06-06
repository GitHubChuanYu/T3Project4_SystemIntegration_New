#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane
from std_msgs.msg import Int32
import tf
import math
from copy import deepcopy

'''
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 1.0


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_lane = None
        self.stopline_wp_idx = -1

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.spin()

    def closest_waypoint(self, curr_pose):
        closest_distance = float('inf')
        closest_idx = -1
        for idx, waypoint in enumerate(self.base_lane):
            distance = self.straight_dist(
                waypoint.pose.pose.position,
                curr_pose.pose.position)
            if distance < closest_distance:
                closest_idx = idx
                closest_distance = distance
        curr_yaw = self.compute_yaw(curr_pose.pose.orientation)

        map_x = self.base_lane[closest_idx].pose.pose.position.x
        map_y = self.base_lane[closest_idx].pose.pose.position.y
        heading = math.atan2(
            (map_y - curr_pose.pose.position.y),
            (map_x - curr_pose.pose.position.x)
        )

        if abs(curr_yaw - heading) > math.pi / 4:
            return closest_idx + 1
        else:
            return closest_idx

    def compute_yaw(self, orientation):
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ]
        )
        return yaw
    
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        closest_idx = self.closest_waypoint(msg)
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        waypoints = deepcopy(self.base_lane[
            closest_idx:closest_idx+LOOKAHEAD_WPS
        ])
        if self.stopline_wp_idx != -1 and self.stopline_wp_idx < farthest_idx:
            waypoints = self.decelerate_waypoints(waypoints, closest_idx)

        final_waypoints_msg = Lane()
        final_waypoints_msg.waypoints = waypoints

        self.final_waypoints_pub.publish(final_waypoints_msg)
        
    def decelerate_waypoints(self, waypoints, closest_idx):
        
        last_idx = self.stopline_wp_idx - closest_idx - 3
        last = waypoints[last_idx]
        last.twist.twist.linear.x = 0.
        for wp in waypoints[:last_idx][::-1]:
            dist = self.straight_dist(
                wp.pose.pose.position, last.pose.pose.position)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0.
            self.set_waypoint_velocity(
                wp, min(vel, self.get_waypoint_velocity(wp)))
        return waypoints

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_lane = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        stopline_wp_idx = msg.data
        if stopline_wp_idx == -1:
            self.stopline_wp_idx = -1
        else:
            self.stopline_wp_idx = stopline_wp_idx

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoint, velocity):
        waypoint.twist.twist.linear.x = velocity
        
    def straight_dist(self, pos0, pos1):
        return math.sqrt((pos0.x - pos1.x) ** 2 + (pos0.y - pos1.y) ** 2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
