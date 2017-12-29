#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
UPDATE_FREQUENCY = 50 #FIXME

def closest_waypoint(current_pose, waypoints):
    dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
    min_dist = 0xFFFFFFF
    closest_pt_id = 0
    for i in range(len(waypoints)):
        dist = dl(current_pose.pose.position, waypoints[i].pose.pose.position)
        if min_dist > dist:
            min_dist = dist
            closest_pt_id = i

    return closest_pt_id

def next_waypoint_id(current_pose, waypoints):
    closest_pt_id = closest_waypoint(current_pose, waypoints)

    pos = current_pose.pose.position
    orientation = current_pose.pose.orientation
    _, _, theta = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    next_pos = waypoints[closest_pt_id].pose.pose.position
    heading = np.arctan2((next_pos.y - pos.y), (next_pos.x - pos.x))
    angle = np.abs(theta - heading)

    if angle > np.pi/4.0:
        closest_pt_id += 1

    return closest_pt_id

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint', 'TODO', self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', 'TODO', self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.current_pose = None

        rate = rospy.Rate(UPDATE_FREQUENCY) # 50Hz
        while not rospy.is_shutdown():
            self.publish()
            if self.base_waypoints is not None:
                rospy.loginfo("waypoints: {}".format(len(self.base_waypoints.waypoints)))
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        rospy.loginfo("current_pose: {}".format(msg.pose.position.x))
        self.current_pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        '''distance(self, waypoints, wp1, wp2): Computes the distance between two waypoints in a list along the piecewise linear arc connecting all waypoints between the two. Here, waypoints is a list of waypoints, and wp1 and wp2 are the indices of two waypoints in the list. This method may be helpful in determining the velocities for a sequence of waypoints leading up to a red light (the velocities should gradually decrease to zero starting some distance from the light).
        '''
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish(self):
        if self.base_waypoints and self.current_pose:
            final_waypoints = Lane()
            next_id = next_waypoint_id(self.current_pose, self.base_waypoints.waypoints)
            # TODO build final_waypoints
            for i in range(LOOKAHEAD_WPS):
                final_waypoints.waypoints.append(self.base_waypoints.waypoints[next_id + i])
            self.final_waypoints_pub.publish(final_waypoints)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
