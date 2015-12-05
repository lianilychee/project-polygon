#!/usr/bin/env python
"""
Script for controlling the robots using the values from the gui
Run 'rosrun rqt_reconfigure rqt_reconfigure' to bring up the gui
"""

import rospy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from project_polygon.cfg import DriverConfig

class MultiController(object):
    """
    Node for controlling multiple robots using the dynamic gui
    """

    def __init__(self):
        """
        Initialize node
        """
        rospy.init_node('multirobot_gui', anonymous = True)

        # Config server
        self.srv = Server(DriverConfig, self.callback)

        # Publishers for (up to) 4 robots
        self.pub1 = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=10)
        self.pub3 = rospy.Publisher('robot3/cmd_vel', Twist, queue_size=10)
        self.pub4 = rospy.Publisher('robot4/cmd_vel', Twist, queue_size=10)

        # X velocities for (up to) 4 robots
        self.x1 = 0
        self.x2 = 0
        self.x3 = 0
        self.x4 = 0

    def callback(self, config, level):
        """
        Get the values from the gui
        """
        self.x1 = config['robot1_xvel']
        self.x2 = config['robot2_xvel']
        self.x3 = config['robot3_xvel']
        self.x4 = config['robot4_xvel']

        self.z1 = config['robot1_zang']
        self.z2 = config['robot2_zang']
        self.z3 = config['robot3_zang']
        self.z4 = config['robot4_zang']

        # Must return config in callback
        return config

    def run(self):
        """
        The main run loop, publish twist messages
        """
        r = rospy.Rate(5)
        twist1 = Twist()
        twist2 = Twist()
        twist3 = Twist()
        twist4 = Twist()
        while not rospy.is_shutdown():
            # Publish each twist message
            twist1.linear.x = self.x1
            twist1.angular.z = self.z1

            twist2.linear.x = self.x2
            twist2.angular.z=self.z1

            twist3.linear.x = self.x3
            twist3.angular.z =self.z3

            twist4.linear.x = self.x4
            twist4.angular.z=self.z4


            self.pub1.publish(twist1)
            self.pub2.publish(twist2)
            self.pub3.publish(twist3)
            self.pub4.publish(twist4)
            r.sleep()

if __name__ == '__main__':
    node = MultiController()
    node.run()
