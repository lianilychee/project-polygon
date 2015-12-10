#!/usr/bin/env python

"""
Listens for packets from the Omni node. Calculates its velocity based on the
information in the packets.
"""

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from project_polygon.msg import Packet
import numpy as np
import math
import helper_funcs as hp

# assume that we are receiving an info packet from Omni
# info packet has: R, centroid, k_a, k_b, k_c, location,
#   number of bots within region, other bot locations


class Agent:
    def __init__(self, i):
        """
        Initialize node. Takes in an index (e.g. '1')
        """
        rospy.init_node('agent', anonymous=True)

        # previous state for velocity calculation
        self.last_stamp = rospy.Time.now()
        self.xy_vel = np.array([0.0, 0.0])

        # odom data (now using ceiling markers)
        self.x = None
        self.y = None
        self.yaw = None

        # packet data
        self.stamp = None
        self.centroid = None
        self.R = None
        self.k_a = None
        self.k_b = None
        self.k_c = None
        self.bot_qty = None
        self.bot_pos = None

        # cmd_vel command and proportional constants
        self.command = Twist()
        self.k_px = 1.0
        self.k_pz = 1.0

        rospy.Subscriber('robot{}/packet'.format(i), Packet, self.assign_data)
        rospy.Subscriber('robot{}/STAR_pose_continuous'.format(i), PoseStamped, self.assign_odom)
        self.pub = rospy.Publisher('robot{}/cmd_vel'.format(i), Twist, queue_size=10)

    def assign_odom(self, msg):
        """
        Save x, y, and yaw from odometry data (now using ceiling markers)
        """
        pose = msg.pose
        self.x, self.y, self.yaw = hp.convert_pose_to_xy_and_theta(pose)

    def assign_data(self, data):
        """
        Unpack data packet and assign to self.<attr>
        """
        self.stamp = data.header.stamp
        self.centroid = data.centroid
        self.R = data.R
        self.k_a = data.k_a
        self.k_b = data.k_b
        self.k_c = data.k_c
        self.bot_qty = data.n  # number of robots it knows about (incl. self)
        self.bot_pos = data.others  # list of positions (excluding self)

    def calc_vels(self):
        """
        Calculate velocities (resultant, linear, angular) from
        data packet attributes.
        """

        # dist betw Bot and centroid
        dci = hp.euclid_dist((self.x, self.y), self.centroid)

        # x- and y-accel of Bot to centroid
        acc_xci = -self.k_c * (dci-self.R) * (1/dci) * (self.x-self.centroid[0])
        acc_yci = -self.k_c * (dci-self.R) * (1/dci) * (self.y-self.centroid[1])

        # define x- and y-accel betw Bot and other bots
        acc_xij = 0.0
        acc_yij = 0.0

        # calculate L based on how many bots it knows about
        L = 2 * self.R * math.sin(math.pi/self.bot_qty)

        for j in range(len(self.bot_pos)):
            botj_xy = (self.bot_pos[j].position.x, self.bot_pos[j].position.y)

            # dist betw Bot and bot
            dij = hp.euclid_dist((self.x, self.y), botj_xy)

            # calc x- and y-accel betw Bot and bot j
            if dij <= L:
                acc_xij += -self.k_a * (dij-L) * (1/dij) * (self.x-botj_xy[0])
                acc_yij += -self.k_a * (dij-L) * (1/dij) * (self.y-botj_xy[1])

        # update accel
        acc = np.array((acc_xci+acc_xij - self.k_b*self.xy_vel[0],
                        acc_yci+acc_yij - self.k_b*self.xy_vel[1]))

        # update vel using time since last update as timestep
        tau = (self.stamp - self.last_stamp).to_sec()
        self.last_stamp = self.stamp

        # new velocity = old velocity + (acceleration * timestep)
        self.xy_vel = self.xy_vel + (acc * tau)

        self.xy_vel[self.xy_vel > 0.5] = 0.5  # set neato vel upper bound
        self.xy_vel[self.xy_vel < -0.5] = -0.5  # set neato vel lower bound

    def move_bot(self):
        """
        Move the robot using velocities from calc_vels()
        """
        # do nothing if no packet or odom data has been received yet
        if self.stamp is None or self.x is None:
            return

        self.calc_vels()    # sets self.xy_vel

        # using complex number to represent vector
        res_vel = complex(*self.xy_vel)

        magnitude = abs(res_vel)
        angle = np.angle(res_vel)
        angle_diff = hp.angle_diff(angle, self.yaw)

        # convert xy_vel parallel to bot heading component -> linear velocity
        l_vel = magnitude * math.cos(angle_diff)

        # convert xy_vel perpendicular to bot heading -> angular velocity
        a_vel = magnitude * math.sin(angle_diff)

        # prevent bot from driving backwards: stop linear velocity and rotate
        if l_vel < 0:
            l_vel = 0
            a_vel = math.copysign(0.5, angle_diff)

        self.command.linear.x = self.k_px * l_vel
        self.command.angular.z = self.k_pz * a_vel
        self.pub.publish(self.command)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.move_bot()
            r.sleep()

if __name__ == '__main__':
    import sys
    i = int(sys.argv[1])
    node = Agent(i)
    node.run()
