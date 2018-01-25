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
MAX_DEACCEL =  8 # m/s^2, this is a observed value after experiments in the simulator
MAX_JERK    = 10 # m/s^3
REF_VEL_RATIO_TO_MAX = 0.95
LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
UPDATE_FREQUENCY = 50 #FIXME
BRAKING_SAFETY_MARGIN = 5

# state
STATE_STR = ["stop", "accel", "decel", "const_speed"]
STATE_STOP = 0
STATE_ACCEL = 1
STATE_DEACCEL = 2
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

    return closest_pt_id

def distance(x1, y1, x2, y2):
    return np.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

# Transform from Cartesian x,y coordinates to Frenet s,d coordinates
def getFrenet(pose, waypoints):
    next_wp = next_waypoint_id(pose, waypoints);

    prev_wp = next_wp - 1
    if next_wp == 0:
        prev_wp  = len(waypoints) - 1

    n_x = waypoints[next_wp].pose.pose.position.x - waypoints[prev_wp].pose.pose.position.x
    n_y = waypoints[next_wp].pose.pose.position.y - waypoints[prev_wp].pose.pose.position.y
    x_x = pose.pose.position.x - waypoints[prev_wp].pose.pose.position.x
    x_y = pose.pose.position.y - waypoints[prev_wp].pose.pose.position.y

    # find the projection of x onto n
    proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    proj_x = proj_norm*n_x;
    proj_y = proj_norm*n_y;

    frenet_d = distance(x_x, x_y, proj_x, proj_y);

    # see if d value is positive or negative by comparing it to a center point

    center_x = 1000 - waypoints[prev_wp].pose.pose.position.x
    center_y = 2000 - waypoints[prev_wp].pose.pose.position.y
    centerToPos = distance(center_x, center_y, x_x, x_y);
    centerToRef = distance(center_x, center_y, proj_x, proj_y);

    if centerToPos <= centerToRef:
       frenet_d *= -1;

    # calculate s value
    frenet_s = 0;
    for i in range(prev_wp):
        frenet_s += distance(   waypoints[i].pose.pose.position.x,
                                waypoints[i].pose.pose.position.y,
                                waypoints[i+1].pose.pose.position.x,
                                waypoints[i+1].pose.pose.position.y);
    frenet_s += distance(0, 0, proj_x, proj_y);

    return frenet_s, frenet_d

def convert_waypoints_to_frenetpoints(waypoints):
    maps_s = [0.0]
    frenet_s = 0;
    for i in range(len(waypoints) - 1):
        frenet_s += distance(   waypoints[i].pose.pose.position.x,
                                waypoints[i].pose.pose.position.y,
                                waypoints[i+1].pose.pose.position.x,
                                waypoints[i+1].pose.pose.position.y);
        maps_s.append(frenet_s)
    return maps_s

# Transform from Frenet s,d coordinates to Cartesian x,y
def getXY(s, d, maps_s, waypoints):
    prev_wp = -1;

    while (s > maps_s[prev_wp+1] and (prev_wp < (len(maps_s) - 1) )):
        prev_wp += 1

    wp2 = (prev_wp+1)%len(waypoints)

    heading = np.arctan2(   (waypoints[wp2].pose.pose.position.y - waypoints[prev_wp].pose.pose.position.y),
                            (waypoints[wp2].pose.pose.position.x - waypoints[prev_wp].pose.pose.position.x))
    # the x,y,s along the segment
    seg_s = (s-maps_s[prev_wp])

    seg_x = waypoints[prev_wp].pose.pose.position.x + seg_s*np.cos(heading)
    seg_y = waypoints[prev_wp].pose.pose.position.y + seg_s*np.sin(heading)

    perp_heading = heading-pi()/2;

    x = seg_x + d*np.cos(perp_heading);
    y = seg_y + d*np.sin(perp_heading);

    return {x,y}

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
        rospy.init_node('waypoint_updater')

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

        rate = rospy.Rate(UPDATE_FREQUENCY) # 50Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.loginfo("current_pose: {}".format(msg.pose.position.x))
        self.current_pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        for i in range(len(self.base_waypoints.waypoints)):
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

    def update_waypoints(self, waypoints, current_wp, target_vel, vel0=None, accel=None):
        rospy.loginfo("update_waypoints({}, {}, {}".format(target_vel, vel0, accel))
        rospy.loginfo("\tstate: {}".format(STATE_STR[self.state]))
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        if vel0 is not None and accel is not None:
            initial_dist = dl(self.current_pose.pose.position, self.base_waypoints.waypoints[current_wp].pose.pose.position)
            vel = vel0
            for i in range(current_wp, current_wp + LOOKAHEAD_WPS):
                dist = self.distance(waypoints, current_wp, i)
                if vel != target_vel:
                    vel = cal_vel_from_dist(initial_dist + dist, vel0, accel)
                    if np.abs(vel - target_vel) < EPS:
                        vel = target_vel
                if accel >= 0 and vel >= target_vel:
                    vel = target_vel
                elif accel < 0 and vel <= target_vel:
                    vel = target_vel

                idx = i%len(self.base_waypoints.waypoints)
                self.set_waypoint_velocity(waypoints, idx, vel)
                rospy.loginfo("{}, {}".format(i, vel))
        else:
            for i in range(current_wp, current_wp + LOOKAHEAD_WPS):
                idx = i%len(self.base_waypoints.waypoints)
                self.set_waypoint_velocity(waypoints, idx, target_vel)


    def update_state(self, current_wp):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        if current_wp > 0:
            # FIXME
            self.last_vel = self.base_waypoints.waypoints[(current_wp - 1)].twist.twist.linear.x
        if self.state == STATE_STOP:
            self.last_vel = 0

        initial_dist = dl(self.current_pose.pose.position, self.base_waypoints.waypoints[current_wp].pose.pose.position)
        rospy.loginfo("#####[{}]#####, {}m/s".format(STATE_STR[self.state], self.last_vel))
        rospy.loginfo("current/traffic_waypoint: {}/{}".format(current_wp, self.traffic_waypoint))
        if self.last_pose:
            pose_diff = dl(self.last_pose.pose.position, self.current_pose.pose.position)
            rospy.loginfo("pose diff: {}".format(pose_diff))
        if self.traffic_waypoint and self.traffic_waypoint is not -1:
            dist_from_stopline = initial_dist + self.distance(self.base_waypoints.waypoints, current_wp, self.traffic_waypoint)
            if self.state == STATE_CONST_SPEED or self.state == STATE_ACCEL:
                max_dist_for_brake = 0.5*(self.last_vel*self.last_vel)/self.ref_decel + BRAKING_SAFETY_MARGIN
                min_dist_for_brake = 0.5*(self.last_vel*self.last_vel)/MAX_DEACCEL + BRAKING_SAFETY_MARGIN
                if dist_from_stopline < max_dist_for_brake and dist_from_stopline > min_dist_for_brake:
                    self.state = STATE_DEACCEL

                    decel = -0.5*(self.last_vel*self.last_vel)/(dist_from_stopline - BRAKING_SAFETY_MARGIN)
                    self.update_waypoints(self.base_waypoints.waypoints, current_wp, 0.0, self.last_vel, decel)
                else:
                    rospy.loginfo("a red light is detected but we can go a bit further.")
                    rospy.loginfo("{}, {}, {}".format(dist_from_stopline, min_dist_for_brake, max_dist_for_brake))

        elif self.state == STATE_ACCEL and self.last_vel == self.ref_vel:
            self.state = STATE_CONST_SPEED
            self.update_waypoints(self.base_waypoints.waypoints, current_wp, self.ref_vel)

        elif self.state == STATE_STOP or self.state == STATE_DEACCEL:
            self.state = STATE_ACCEL
            self.update_waypoints(self.base_waypoints.waypoints, current_wp, self.ref_vel, self.last_vel, self.ref_accel)

        elif self.state == STATE_CONST_SPEED:
            self.update_waypoints(self.base_waypoints.waypoints, current_wp, self.ref_vel)

        if self.state == STATE_DEACCEL and pose_diff < EPS:
            self.state = STATE_STOP
        return

    def publish(self):
        if self.base_waypoints and self.current_pose:
            final_waypoints = Lane()
            current_wp = next_waypoint_id(self.current_pose, self.base_waypoints.waypoints)
            # state management
            self.update_state(current_wp)
            # TODO build final_waypoints
            for i in range(current_wp, current_wp + LOOKAHEAD_WPS):
                idx = i%len(self.base_waypoints.waypoints)
                final_waypoints.waypoints.append(self.base_waypoints.waypoints[idx])
            self.final_waypoints_pub.publish(final_waypoints)
            rospy.loginfo("final_waypoints.waypoints[0].twist.twist.linear.x: {}".format(final_waypoints.waypoints[0].twist.twist.linear.x))

            self.last_pose = self.current_pose

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
