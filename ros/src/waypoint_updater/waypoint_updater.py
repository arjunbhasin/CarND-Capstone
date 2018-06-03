#!/usr/bin/env python
# Team: RunCarla
# 

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

from scipy.spatial import KDTree

import math

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



'''
Instructions from lesson:

1. First Version 
The goal for the first version of the node should be simply to subscribe to the topics

/base_waypoints & /current_pose

and publish a list of waypoints to /final_waypoints
The /base_waypoints topic publishes a list of all waypoints for the track, 
so this list includes waypoints both before and after the vehicle 
(note that the publisher for /base_waypoints publishes only once). 
For this step in the project, the list published to /final_waypoints should include just a 
fixed number of waypoints currently ahead of the vehicle.

Topic : Msg Type
/base_waypoints : styx_msgs/Lane  
/current_pose  : geometry_msgs/PoseStamped 
/final_waypoints  :  styx_msgs/Lane


2. Final version
Once traffic light detection is working properly, you can incorporate the traffic light data into your waypoint
updater node. To do this, you will need to add a subscriber for the /traffic_waypoint topic and implement the 
traffic_cb callback for this subscriber.

It will be up to you determine what the deceleration should be for the vehicle, 
and how this deceleration corresponds to waypoint target velocities. 
As in the Path Planning project, acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3.

Important:
Be sure to limit the top speed of the vehicle to the km/h velocity set by the velocity rosparam in waypoint_loader. 
Reviewers will test on the simulator with an adjusted top speed prior to testing on Carla.

'''

# Rate in Hz
PUBLISHING_RATE = 25
# Number of waypoints we will publish. You can change this number
# Warning: Having large waypoints can increase latency
LOOKAHEAD_WPS = 200 
# Max deceleration 
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb, queue_size=1)

        # Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        # Gives us control over the publishing frequency
        self.loop()

    # Infinite loop
    def loop(self):
        rate = rospy.Rate(PUBLISHING_RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

    # Get the closest waypoint index
    def get_closest_waypoint_idx(self):
        # Current pose (x,y)
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # Get the index of the closest way point
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]

        # Check if point is infront or behind the car
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx+1) % len(self.waypoints_2d)

        return closest_idx

    # Publishes the final lane
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)


    def generate_lane(self):
        lane = Lane()
        # Get the closest index
        closest_idx = self.get_closest_waypoint_idx()
        # Set the farthest index
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]

        # If no traffic light was detected, publish the base_waypoints as it is
        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    # Important function
    # Warning: Do not modify the original base_waypoints list (that message comes only once)
    def decelerate_waypoints(self, base_waypoints, closest_idx):
        temp = []
        for i , wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # 2 is subtracted to ensure that the front of the car stops at the stop line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            # Sum of distance between waypoints (sum of line segments)
            dist = self.distance(waypoints, i, stop_idx)

            # We can experiment with other functions too as this is quite steep 
            vel = math.sqrt(2 * MAX_DECEL * dist)
            if vel < 1.:
                vel = 0

            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp


    def pose_cb(self, msg):
        self.pose = msg


    # Latched callback, only called once (base waypoints are static)
    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        # Setup the Kd Tree which has log(n) complexity
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        # Not used in this project
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

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