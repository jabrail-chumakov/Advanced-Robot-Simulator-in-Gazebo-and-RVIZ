#! /usr/bin/env python

"""
.. module:: third_option
    :platform: Unix
    :synopsis: Python script representing third option *(self-control with obstacle avoidance)*
.. moduleauthor:: Jabrail Chumakov `rospy <jabrail.chumakov@nu.edu.kz>`_

Packages:
    `NumPy <https://pypi.org/project/numpy/>`_: The fundamental package for array computing with Python.
    
Publishes to:
    ``/cmd_vel``: The desired robot position.
    
Subscribes to:
    ``/new_cmd_vel``: The newly obtained robot position.
    ``/scan``: Scans for obstacles in front of the robot.
"""

import rospy
import numpy
import array as arr
from sensor_msgs.msg import LaserScan           
from geometry_msgs.msg import Twist, Vector3   


def distance_control(ranges):
    """
    Function to check distance from 3 different sides, which returns minimal range at the end.
    
    Args:
        ranges(array): Angles around the robot.
        
    Returns:
        ranges(min_d): Returns array with minimum distance until the obstacle.
        
    .. note::
        1. Bound angles on appropriate sides and store it in mid_d in appropriate order.
        
        2. Stores information about minimal range from 3 sides (right, front, left) in ``mid_d``.
    """
    right_side = ranges[0:240]
    front_side = ranges[240:480]
    left_side = ranges[480:721]
    min_d = arr.array('d', [min(right_side), min(front_side), min(left_side)])
    return min_d

# In case if robot will be too close to wall, there shoud me some restrictions on that called border threshold.
# This value can be regulated in the script, dependent on the user desire
border_th = 0.6
# Creates a Twist message with linear and angular values for 'x', 'y' and 'z'
move_cmd = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))


def obstacle_avoidance(info):
    """
    Function to check whether robot is too close to the obstacle.
    
    .. note::
        1. Creates publisher to publish Twist to ``/cmd_vel``.
        
        2. Takes array with minimal ranges from :func:`distance_control` function.
        
        3. If distance from right is less than ``border_th = 0.6``, then it's not possible to rotate to right further.
        
        4. If distance in front of the robot is less than ``border_th = 0.6``, then it's not possible to move robot forward further.
        
        5. If distance from left is less than ``border_th = 0.6``, then it's not possible to rotate to left further.
        
        6. Publish linear and angular values for **X**, **Y** and **Z**.
    """
    global move_cmd
    movement_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    check_dist = distance_control(info.ranges)
    #flag_control = rospy.get_param("/obstacle_avoidance")
    if check_dist[0] < border_th:
        if move_cmd.angular.z < 0:
            move_cmd.angular.z = 0    

    if check_dist[1] < border_th:
        if move_cmd.linear.x > 0:
            move_cmd.linear.x = 0
    
    if check_dist[2] < border_th:
        if move_cmd.angular.z > 0:
            move_cmd.angular.z = 0

    movement_publisher.publish(move_cmd)


def obstacle_avoidance_corrected(info):
    """
    Function to rewrite ``move_cmd`` with linear and angular values from ``/new_cmd_vel`` topic.
    """
    global move_cmd
    move_cmd = info

if __name__ == "__main__":
    """
    Initializes ``art_node`` node and subscribes to ``/new_cmd_vel`` with :func:`obstacle_avoidance_corrected` callback, as well as to ``/scan`` with :func:`obstacle_avoidance` callback.
    """
    # Initialize node
    rospy.init_node('art_node')
    # Subscribes to '/new_cmd_vel' topic
    rospy.Subscriber("/new_cmd_vel", Twist, obstacle_avoidance_corrected)
    # Subscribes to '/scan' topic
    rospy.Subscriber("/scan", LaserScan, obstacle_avoidance)
    rospy.spin()