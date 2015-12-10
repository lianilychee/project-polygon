#!/usr/bin/env python

"""
Instantiates all agents, and knows all information of regarding all agents.
Reconcile agent base_links to world coordinate frame.
Sends packet information to each agent for individual path-planning.
"""

import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import helper_funcs as hp
from copy import deepcopy
from project_polygon.msg import Packet


class Omni:
    def __init__(self, n):
        """
        Omni init class sets the variables, gets the robots' position, and
        publishes to packet for instances of Agent to subscribe to.
        """

        rospy.init_node('omniscient')

        # set all constants
        self.centroid = (0.8, -1.4)
        self.k_a = 0.2
        self.k_b = 0.2
        self.k_c = 0.08
        self.R = 0.75

        self.sensing_radius = 5  # sensing radius of each robot

        # create empty list with a size of num of robots
        self.bot_pos = [None] * n

        # copy of bot positions to guard against threading shenanigans
        self.bot_pos_copy = [None] * n

        # subscribe to all bots and create packet publishers for each
        self.pub = []
        for i in range(n):
            rospy.Subscriber('/robot{}/STAR_pose_continuous'.format(i), PoseStamped, self.get_pos, callback_args=i)
            self.pub.append(rospy.Publisher('/robot{}/packet'.format(i), Packet, queue_size=10))

    def get_pos(self, msg, callback_args):
        """
        call back function to get the position of robots to deploy to the agents.
        """
        self.bot_pos[callback_args] = msg.pose

    def neighbor_bots(self, i):
        """
        Take in a bot index and returns a list of Poses for all
        other bots within self.sensing_radius
        """
        neighbors = []
        for j in range(len(self.bot_pos_copy)):
            if (j != i) and (hp.pose_euclid_dist(self.bot_pos_copy[i], self.bot_pos_copy[j]) <= self.sensing_radius):
                neighbors.append(self.bot_pos_copy[j])

        return neighbors

    def send_pkt(self, i):
        """
        Define packet contents, then publish to appropriate topic for robot i
        """
        neighbors = self.neighbor_bots(i)

        my_packet = Packet(
            centroid = self.centroid,
            R = self.R,
            k_a = self.k_a,
            k_b = self.k_b,
            k_c = self.k_c,
            n = len(neighbors) + 1,
            others = neighbors
        )
        my_packet.header.stamp = rospy.Time.now()

        self.pub[i].publish(my_packet)

    def run(self):
        # pass
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if not None in self.bot_pos:
                self.bot_pos_copy = deepcopy(self.bot_pos)
                for i in range(len(self.bot_pos)):
                    self.send_pkt(i)
            r.sleep()


if __name__ == '__main__':
    import sys
    n = int(sys.argv[1])
    node = Omni(n)
    node.run()
