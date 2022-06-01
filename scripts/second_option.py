#! /usr/bin/env python

"""
.. module:: second_option
    :platform: Unix
    :synopsis: Python script representing second option *(self-control without obstacle avoidance)*
.. moduleauthor:: Jabrail Chumakov `rospy <jabrail.chumakov@nu.edu.kz>`_

Packages:
    `Termcolor <https://pypi.org/project/termcolor/>`_: ANSII Color formatting for output in terminal.
    
Service:
    /self_driving Service for ``automatic`` driving by the robot
"""

import os   
import rospy
from termcolor import colored
from final_assignment.srv import TTS_server	


def clearConsole():
    """ 
    Function to clear console after each action to keep it clean and readable in the console window.
    """
    command = 'clear'
    if os.name in ('nt', 'dos'):
        command = 'cls'
    os.system(command)
    
# 
def goal_execute(desired):
    """
    Function which decides which sub-option to choose between self-driving and advanced self-driving received from :mod:`main_menu`.
    
    .. note::
        1. If in :mod:`main_menu` user choosed 2nd option, then (from if-elif statements) second option will be opened.
        
        2. If in :mod:`main_menu` user choosed 3rd option, then (from if-elif statements) third option will be opened.
    """
    if desired.sub_option == 0:
       os.system("roslaunch final_assignment second_option_teleop.launch") 
       print(colored("\nSelf driving was cancelled! Returning to the main menu in 5 seconds!", "yellow", attrs = ["blink"]))
       rospy.sleep(5)
       clearConsole()
    
    elif desired.sub_option == 1:
        os.system("roslaunch final_assignment third_option_teleop.launch") 
        print(colored("\nAdvanced self driving was cancelled! Returning to the main menu in 5 seconds!", "yellow", attrs = ["blink"]))
        rospy.sleep(5)
        clearConsole()
    return 0         

if __name__ == "__main__":
    """
    Initializes ``user_robot_control`` node and declares new ``self_driving`` server with service class ``TTS_server`` and handler as :func:`goal_execute` function.
    """
    # Initializes node 
    rospy.init_node('user_robot_control')
    # Declares new service
    service_self_driving = rospy.Service('self_driving', TTS_server, goal_execute)
    rospy.spin()
