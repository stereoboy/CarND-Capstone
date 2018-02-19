#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
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

EPS=1e-3
KMPH_TO_MPS = 0.277778
# It will be up to you determine what the deceleration should be for the vehicle, and how this deceleration corresponds to waypoint target velocities. 
# As in the Path Planning project, acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3.
MAX_ACCEL   = 10 # m/s^2
MAX_DEACCEL = 10 # m/s^2, this is a observed value after experiments in the simulator
MAX_JERK    = 10 # m/s^3
REF_VEL_RATIO_TO_MAX = 0.95
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
UPDATE_FREQUENCY = 50 #FIXME
BRAKING_SAFETY_MARGIN = 3

# state
STATE_STR = ["STOP", "ACCEL", "DECEL", "CONST"]
STATE_STOP = 0
STATE_ACCEL = 1
STATE_DECEL = 2
STATE_CONST_SPEED = 3

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

    return closest_pt_id%len(waypoints)

def distance(x1, y1, x2, y2):
    return np.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

def cal_vel_from_dist(dist, v0, accel):
    def get_root(a, b, c):
        if b*b - 4*a*c > 0:
            return (-b + np.sqrt(b*b - 4*a*c))/(2*a)
        else:
            return (-b)/(2*a)


    # dist = 0.5*accel*t^2 + v0*t
    t = get_root(0.5*accel, v0, -dist)

    return accel*t + v0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.INFO)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', 'TODO', self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.current_pose = None
        self.last_pose = None
        self.traffic_waypoint = None
        self.max_vel = KMPH_TO_MPS*rospy.get_param('/waypoint_loader/velocity')
        self.last_vel = 0.0
        self.ref_vel = self.max_vel*REF_VEL_RATIO_TO_MAX
        self.ref_accel = 4.0
        self.ref_decel = 4.0
        self.state = STATE_STOP
        self.lookahead_wps = LOOKAHEAD_WPS
        self.start_time = rospy.get_time()

        rate = rospy.Rate(UPDATE_FREQUENCY) # 50Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        self.base_waypoints_num = len(self.base_waypoints.waypoints)
        self.lookahead_wps = np.minimum(self.base_waypoints_num, LOOKAHEAD_WPS)
        for i in range(self.base_waypoints_num):
            self.set_waypoint_velocity(self.base_waypoints.waypoints, i, 0.0)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data

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

    def update_waypoints(self, waypoints, current_pose, current_wp, target_vel, vel0=None, accel=None):
        rospy.logdebug("update_waypoints({}, {}, {})".format(target_vel, vel0, accel))
        rospy.logdebug("\tstate: {}".format(STATE_STR[self.state]))
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        if vel0 is not None and accel is not None:
            initial_dist = dl(current_pose.pose.position, waypoints[current_wp].pose.pose.position)
            vel = vel0
            for i in range(current_wp, current_wp + self.lookahead_wps):
                idx = i%len(waypoints)
                dist = self.distance(waypoints, current_wp, idx)
                if vel != target_vel:
                    vel = cal_vel_from_dist(initial_dist + dist, vel0, accel)
                    if np.abs(vel - target_vel) < EPS:
                        vel = target_vel
                if accel >= 0 and vel >= target_vel:
                    vel = target_vel
                elif accel < 0 and vel <= target_vel:
                    vel = target_vel

                self.set_waypoint_velocity(waypoints, idx, vel)
                rospy.logdebug("{}, {}".format(idx, vel))
        else:
            for i in range(current_wp, current_wp + self.lookahead_wps):
                idx = i%self.base_waypoints_num
                self.set_waypoint_velocity(waypoints, idx, target_vel)


    def update_state(self, current_wp):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        if current_wp > 0:
            # FIXME
            self.last_vel = self.base_waypoints.waypoints[(current_wp - 1)].twist.twist.linear.x
        if self.state == STATE_STOP:
            self.last_vel = 0

        initial_dist = dl(self.current_pose.pose.position, self.base_waypoints.waypoints[current_wp].pose.pose.position)
        rospy.logdebug("-----[{}]-----, {}m/s".format(STATE_STR[self.state], self.last_vel))
        rospy.logdebug("elapsed time: %.3f sec"%((rospy.get_time() - self.start_time)))
        rospy.logdebug("current/traffic_waypoint: {}/{}".format(current_wp, self.traffic_waypoint))
        if self.state == STATE_DECEL:
            rospy.logdebug("self.decel: {}".format(self.decel))

        if self.last_pose:
            if self.last_pose != self.current_pose: # if updated
                pose_diff = dl(self.last_pose.pose.position, self.current_pose.pose.position)
            else:
                pose_diff = None
            rospy.logdebug("pose diff: {}".format(pose_diff))

        if self.traffic_waypoint and self.traffic_waypoint is not -1:
            # calculate a distance from the given stopline
            if self.traffic_waypoint >= current_wp:
                dist_from_stopline = initial_dist + self.distance(self.base_waypoints.waypoints, current_wp, self.traffic_waypoint)
            else:
                dist_from_stopline = initial_dist + self.distance(self.base_waypoints.waypoints, current_wp, (self.base_waypoints_num - 1)) + self.distance(self.base_waypoints.waypoints, 0, self.traffic_waypoint)

            # check the current status (current velocity, reference decel, max/min decel
            if self.state == STATE_CONST_SPEED or self.state == STATE_ACCEL:
                max_dist_for_brake = 0.5*(self.last_vel*self.last_vel)/self.ref_decel + BRAKING_SAFETY_MARGIN
                min_dist_for_brake = 0.5*(self.last_vel*self.last_vel)/MAX_DEACCEL + BRAKING_SAFETY_MARGIN
                if dist_from_stopline < max_dist_for_brake and dist_from_stopline > min_dist_for_brake:
                    self.state = STATE_DECEL

                    decel = -0.5*(self.last_vel*self.last_vel)/(dist_from_stopline - BRAKING_SAFETY_MARGIN)
                    self.update_waypoints(self.base_waypoints.waypoints, self.current_pose, current_wp, 0.0, self.last_vel, decel)
                    self.decel = decel
                elif dist_from_stopline > max_dist_for_brake:
                    rospy.logdebug("a red light is detected but we can go a bit further.")
                    rospy.logdebug("{}, {}, {}".format(dist_from_stopline, min_dist_for_brake, max_dist_for_brake))
                else:
                    rospy.logdebug("a red light is detected but we can not stop with the maximum decel.")
                    rospy.logdebug("{}, {}, {}".format(dist_from_stopline, min_dist_for_brake, max_dist_for_brake))
        elif self.traffic_waypoint == None:
            # do nothing, stay
            pass

        else: # self.traffic_waypoint == -1
            if self.state == STATE_ACCEL and self.last_vel == self.ref_vel: 
                self.state = STATE_CONST_SPEED
                self.update_waypoints(self.base_waypoints.waypoints, self.current_pose, current_wp, self.ref_vel)

            elif self.state == STATE_STOP or self.state == STATE_DECEL:
                self.state = STATE_ACCEL
                self.update_waypoints(self.base_waypoints.waypoints, self.current_pose, current_wp, self.ref_vel, self.last_vel, self.ref_accel)


        if self.state == STATE_CONST_SPEED:
            self.update_waypoints(self.base_waypoints.waypoints, self.current_pose, current_wp, self.ref_vel)

        if self.state == STATE_DECEL and pose_diff is not None and pose_diff < EPS:
            self.state = STATE_STOP
        return

    def publish(self):
        # TODO build final_waypoints
        if self.base_waypoints and self.current_pose:
            final_waypoints = Lane()
            current_wp = next_waypoint_id(self.current_pose, self.base_waypoints.waypoints)
            # state management
            self.update_state(current_wp)
            for i in range(current_wp, current_wp + self.lookahead_wps):
                idx = i%self.base_waypoints_num
                final_waypoints.waypoints.append(self.base_waypoints.waypoints[idx])
            self.final_waypoints_pub.publish(final_waypoints)
            rospy.logdebug("final_waypoints.waypoints[0].twist.twist.linear.x: {}".format(final_waypoints.waypoints[0].twist.twist.linear.x))

            self.last_pose = self.current_pose

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
