#!/usr/bin/env python

"""
Script for creating multiple robots in the Gazebo simulator.
Takes in a number indicating how many robots to create.
Example: python gazebo_multi_connect.py 3
"""

import subprocess
import shlex
import sys
import time
import termios
import helper_funcs as hp

def create_world():
    """
    Use subprocess to call
    'roslaunch project_polygon gazebo_create_world.launch',
    return process so it can be killed later
    """    
    cmd = ['roslaunch', 'project_polygon', 'gazebo_create_world.launch']

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return proc

def spawn_agent(namespace, x, y):
    """
    Use subprocess to call
    'roslaunch project_polygon gazebo_spawn_agent.launch'
    with the given parameter, return process so it can be killed later
    """
    cmd = ['roslaunch', 'project_polygon', 'gazebo_spawn_agent.launch']
    args = 'robot:={} x:={} y:={}'.format(namespace, x, y)
    cmd.extend(shlex.split(args))

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return proc


if __name__ == '__main__':
    n = int(sys.argv[1])
    processes = []

    # Create world once
    p = create_world()
    processes.append(p)
    print 'Creating world, sleeping for 5 seconds before spawning agents'
    # Agents fail to spawn if done immediately
    time.sleep(5)

    # Spawn each agent
    x = 0
    y = 0
    for i in range(n): 
        namespace = 'robot{}'.format(i)
        p = spawn_agent(namespace, x, y)
        processes.append(p)
        print 'Spawning {}'.format(namespace)
        y += 1

    print 'Please wait for all agents to spawn before running any commands'
    print 'Ctrl-C to quit and close Gazebo'

    # Wait for Ctrl-C
    settings = termios.tcgetattr(sys.stdin)
    key = None
    while key != '\x03':
        key = hp.get_key(settings)

    # Terminate the connections
    for proc in processes:
        proc.terminate()
