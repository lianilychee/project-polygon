"""
Helper functions for project_polygon
"""

import tty
import select
import termios
import sys
import math
import numpy as np

def get_key(settings):
    """
    Return pressed key
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def euclid_dist(pt1, pt2):
    """
    given two positions, calculate the euclid_dist
    """
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)
