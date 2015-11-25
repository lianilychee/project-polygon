#!/usr/bin/env python

"""
Listens to odom topic.  Reconcile agent odoms to world coordinate
system.  Publish location of neighbor agents if agents are within a certain 
distance of each other.
"""

import rospyfrom sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point, Vector3
from sensor_msgs.msg import LaserScan, Image
import tf
import numpy as np
import helper_funcs as hp

# assume that we are receiving an info packet from Omni
# info packet has: R, centroid, self.k_a, k_b, k_c, location, # of bots within region + their locations

class Agent:
    def __init__(self):

        rospy.init_node('Agent') # TODO: remember to change name for multiple bots
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.command = Twist()
        rospy.Subscriber('/packet', packet, self.assign_data) # remember to change name for multiple bots
        self.sense_range=3
        self.tau=0.1

    def assign_data(self, data):
        """
        Unpack data packet and assign to self.attributes.
        """

        self.centroid = data.centroid
        self.k_a = data.k_a
        self.k_b = data.k_b
        self.k_c = data.k_c
        self.bot_qty = data.n  #number of robots existing total
        self.bot_pos = data.others  # list of positions
        self.R = data.R


    def calc_vels(self):
        """
        Calculate velocities - resultant, linear, angular - from data packet attributes.
        """

        for i in range(self.bot_qty):
            dci = hp.euclid_dist(self.bot_pos[i], self.centroid)    # dist betw Bot and centroid

            # x- and y-accel of bot i to bot j
            acc_xci = -self.k_c * (dci - self.R) * (1/dci) * (self.bot_pos[i][0] - self.centroid[0])
            acc_yci = -self.k_c * (dci - self.R) * (1/dci) * (self.bot_pos[i][1] - centroid[1])

            # define x- and y-accel betw Bot and other bots
            acc_xij = 0.0
            acc_yij = 0.0

            # find bots within sensing range and calculate L
            n = sum(1 for k in self.bot_pos if hp.euclid_dist(k, self.bot_pos[i]) <= self.sense_range)
            L = 2 * self.R * math.sin(math.pi/self.bot_qty)

            for j in range(len(self.bot_qty)):
                dij = euclid_dist(self.bot_pos[i], self.bot_pos[j])    # dist betw Bot and bot 

                # calc x- and y-accel betw Bot and bot j
                if (i != j) and (dij <= L):
                    acc_xij += -self.k_a * (dij - L) * (1/dij) * (self.bot_pos[i][0] - self.bot_pos[j][0])
                    acc_yij += -self.k_a * (dij - L) * (1/dij) * (self.bot_pos[i][1] - self.bot_pos[j][1])

            # update accels
            acc[i] = (acc_xci + acc_xij - k_b*vel[i][0], acc_yci + acc_yij - self.k_b*vel[i][1])
            vel_temp[i] = vel[i] + acc[i] * self.tau
            pos_temp[i] = self.bot_pos[i] + vel[i] * self.tau

        vel_temp[vel_temp > 0.5] = 0.5  # set neato vel upper bound

        # update positions and velocities
        self.bot_pos = pos_temp
        self.vel = vel_temp

    def move_bot(self):
        """
        move the robot to its new position using velocities from calc_vels()
        """
        res_vel=self.vel[0]+1j*self.vel[1]  #using complex number notation
        angle=np.angle(res_vel)
        magnitude=abs(res_vel)

        angle_diff=abs(angle-current_angle) #for now, assume current angle is consistant with world
        self.command.linear.x = res_vel
        self.command.angular.z = angle_diff
        self.pub.publish(self.command)

    def run(self):
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = Agent()
    node.run