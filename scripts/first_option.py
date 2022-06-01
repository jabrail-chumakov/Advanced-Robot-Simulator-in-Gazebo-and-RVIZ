#! /usr/bin/env python

"""
.. module:: first_option
    :platform: Unix
    :synopsis: Python script representing first option *(autonomous robot driving)*
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

Packages:
    `Pynput <https://pypi.org/project/pynput/>`_: Allows you to control and monitor input devices.
    
    `Tabulate <https://pypi.org/project/tabulate/>`_: Pretty-print tabular data in Python, a library and a command-line utility.
    
    `Inputimeout <https://pypi.org/project/inputimeout/>`_: Multi platform standard input with timeout.
    
Service:
    ``/move_base``: Service for moving the robot.

    ``/navigation``: Service for navigation in the environment.
"""

import os
import sys
import tty
import rospy
import select
import termios
import actionlib
from pynput import keyboard
from termcolor import colored
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from final_assignment.srv import Coordinates 
from inputimeout import inputimeout, TimeoutOccurred
    
# Defines timeout for robot to finish the task
timeout = 60.0
    
def clearConsole():
    """ 
    Function to clear console after each action to keep it clean and readable in the console window.
    """
    command = 'clear'
    if os.name in ('nt', 'dos'):
        command = 'cls'
    os.system(command)
    

def isData():
    """
    Function to make non-blocking input for goal cancellation.
    """
    return select.select([sys.stdin], [], [], timeout) == ([sys.stdin], [], [])
    

def goal_execute(desired):
    """
    Function which receives **X** and **Y** coordinates from :mod:`main_menu` and sends robot to corresponding point.
    
    Args:
        desired(float64 x, float64 y): Data received from ``move_base_msgs``.
        
    Returns:
        bool(True or False): Whether the robot reached its goal or not.
    
    .. note::
        1. Creates the action client and waiting for server to start.
        
        2. Setting our goal for **X** and **Y**.
        
        3. List of all possible states for the robot.
        
        4. Sending robot to a defined goal.
        
        5. Waits for robot to finish its movemement in 60.0 seconds.
        
        6. Checks the status of the robot after it reaches the goal.
    """
    x = desired.x
    y = desired.y
    
    # Creates the action client and waiting for server to start 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    # Setting our goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1
    
    # List of all possible states for the robot
    goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED',
                   'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING',
                   'RECALLED', 'LOST']

    # Sending robot to a defined goal
    client.send_goal(goal)
    print("\nThe robot is moving to a given coordinates (x, y) = (",x,",",y,")")
    
    # Seconds and counter just need to run while loop for 1 time
    seconds = 0
    counter = 1
    
    # Waits for robot to finish its movemement in 60.0 seconds
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        # If 0 < 1 (or if time hasn't run out yet), then it's possible to cancel operation by pressing ESC
        while (seconds < counter):
            print(colored("\nPress 'Esc' to cancel operation or wait until the time is up!", "cyan"))
            if isData():
                c = sys.stdin.read(1)
                # If ESC is pressed then it prints that operation was cancelled, waits for 5 seconds and returns to main menu again
                if c == '\x1b':         # x1b is ESC
                    client.cancel_goal()
                    print(colored("\nOperation was cancelled! Returning to the main menu in 5 seconds!", "yellow", attrs = ["blink"]))
                    rospy.sleep(5)
                    return False
                    break
                # If time hasn't run out yet, but user pressed any other button (except of ESC), then it print that input is incorrect and continue moving
                elif c != '\x1b':
                    print(colored("\nIncorrect input for cancellation", "yellow"))
                    continue
            # Clear buffer of input for cancellation and then waits for 1 second and exit from while loop
            sys.stdin.flush()
            sys.stdout.flush()
            rospy.sleep(1)
            seconds = seconds + 1 
        
        # Here it checks status of our goal
        while (seconds >= counter):
            state = client.get_state()
            # If it succeeded. then prints text below, waits for 5 seconds and returns TRUE to main manu
            if state == GoalStatus.SUCCEEDED:
                print(colored("\nThe robot has reached its goal! Returning to the main menu in 5 seconds!", "green", attrs = ["blink"]))
                rospy.sleep(5)
                return True   
            # If it's not succeeded, then prints in which state robot failed, waits for 5 seconds and returns FALSE to main menu   
            else:
                client.cancel_goal()
                print("\nGoal failed with status: ", str(goal_states[state]))
                print(colored("The robot hasn't reached its goal! Returning to the main menu in 5 seconds!", "red", attrs = ["blink"]))
                rospy.sleep(5)
                return False        
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
if __name__ == "__main__":
    """
    Initializes ``nagivation_test`` node and declares new ``navigation`` server with service class ``Coordinates`` and handler as :func:`goal_execute` function.
    """
    rospy.init_node('navigation_test')
    service_navigation = rospy.Service('navigation', Coordinates, goal_execute)
    rospy.spin()
