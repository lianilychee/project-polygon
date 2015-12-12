#!/usr/bin/env python

"""
Script for connecting to multiple Neatos at once.
Takes in a series of IP addresses.
Example: python multi_connect.py 192.168.17.200 192.168.17.204
If the full IP address is not given, it assumes 192.168 as the prefix.
Example: python multi_connect.py 17.200 17.204
"""

import subprocess
import shlex
import sys
import termios
import helper_funcs as hp

def connect(ip, namespace, port, fps=10):
    """
    Use subprocess to call 'roslaunch project_polygon bringup_multi.launch'
    with the given parameters, return process so it can be killed later
    """
    cmd = ['roslaunch', 'project_polygon', 'bringup_multi.launch']
    args = 'host:={} robot:={} receive_port:={} fps:={}'.format(ip, namespace, port, fps)
    cmd.extend(shlex.split(args))

    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    return proc


if __name__ == '__main__':
    ip_addrs = sys.argv[1:]

    processes = []
    robot_num = 0
    for ip in ip_addrs: 
        # Fill in prefix if necessary
        if ip.count('.') == 1:
            ip = '192.168.{}'.format(ip)

        if ip.count('.') == 3:
            namespace = '/robot{}'.format(robot_num)
            port = 5000 + robot_num
            p = connect(ip, namespace, port)
            processes.append(p)
            print 'Connecting to {}'.format(ip)
            robot_num += 1
        else:
            print 'Invalid IP address, skipping {}'.format(ip)

    print 'Please wait a moment before running any commands'
    print 'Ctrl-C to quit and close connections'

    # Wait for Ctrl-C 
    settings = termios.tcgetattr(sys.stdin)
    key = None
    while key != '\x03':
        key = hp.get_key(settings)

    # Terminate the connections
    for proc in processes:
        proc.terminate()
