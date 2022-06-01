#! /usr/bin/env python

"""
.. module:: main_menu
    :platform: Unix
    :synopsis: Python script representing user interface
.. moduleauthor:: Jabrail Chumakov <jabrail.chumakov@nu.edu.kz>

Packages:
    `Threading <https://pypi.org/project/threaded/>`_: Decorators for running functions in Thread/ThreadPool/IOLoop.
    
    `Tabulate <https://pypi.org/project/tabulate/>`_: Pretty-print tabular data in Python, a library and a command-line utility.
    
    `Termcolor <https://pypi.org/project/termcolor/>`_: ANSII Color formatting for output in terminal.
    
Service:
    ``/navigation``: Service for navigation in the environment.
    
    ``/self_driving``: Service for automatic driving by the robot.
"""

import os
import sys
import rospy
import threading
import subprocess
from tabulate import tabulate
from termcolor import colored
from final_assignment.srv import TTS_server	 
from final_assignment.srv import Coordinates 


def clearConsole():
    """ 
    Function to clear console after each action to keep it clean and readable in the console window.
    """
    command = 'clear'
    if os.name in ('nt', 'dos'):
        command = 'cls'
    os.system(command)


def menu():    
    """
    Main menu function. After entering appropriate option, this string-input will be sent to :func:`main` to match with corresponding function.
    Clears console each time program is executed calling :func:`clearConsole` function.
    
    Returns:
        input(string): Takes string-input and returns it. An option that user chose in the main menu.
        
    .. note::
        After pressing one of the following key buttons corresponding mode will be executed:
        
        **1** — Autonomously reach **X** and **Y** coordinate.
        
        **2** — Drive the robot with the keyboard.
        
        **3** — Drive the robot with avoid collisions feature.
        
        **4** — Close the program.
    """
    clearConsole()
    table = [[u'\u2460',"Autonomously reach 'x' and 'y' coordinate"],
             [u'\u2461',"Drive the robot with the keyboard"],
             [u'\u2462',"Drive the robot with avoid collisions feature"],
             [u'\u2463',"Close the program"]]
    headers = ["Robot control in the environment"]
    print(tabulate(table, headers, tablefmt = "fancy_grid"))
    selected_option = input("Please, select desired option: ")
    return selected_option


def is_number_tryexcept(s):
    """
    Function to check whether entered string-input from :func:`menu` function is float number or not.
    
    Args:
        s(string): Input string by the user.
    
    Returns:
        s(string): Returns True if string is a number (both positive or negative), otherwise False.
    """
    try:
        float(s)
        return True
    except ValueError:
        return False


def first_option():
    """
    Function called in case if user entered first option in :func:`menu`. 
    
    .. note::
        1. Gives a command to robot to move in user-defined coordinates **(X, Y)**.
        
        2. Takes coordinate for **X** and checks whether it number or not using :func:`is_number_tryexcept` function. If it's positive-negative then takes it as correct coordinate.
           Otherwise prints that entered coordinate is invalid.
        
        3. Takes coordinate for **Y** and checks whether it number or not using :func:`is_number_tryexcept` function. If it's positive-negative then takes it as correct coordinate.
           Otherwise prints that entered coordinate is invalid.
         
        4. Checks if the ``/navigation`` service is available and blocks as long as it can't reach the service.
    
        5. Creates the object which communicates with the ``/navigation`` server.
    
        6. If :mod:`first_option` returned True or False, then :func:`clearConsole` called for both cases. But results are checked in :mod:`first_option`.
    """
    print(colored("You choosed first option! Please, enter desired goal for 'x' and 'y' below: ", "green", attrs = ["bold", "underline"]))
    x = input("- Enter 'x' position: ")
    number_x = is_number_tryexcept(x)
    while number_x is not True:
        print(colored("Invalid coordinate for 'x'", "yellow"))
        x = input("\n- Enter 'x' position: ")
        number_x = is_number_tryexcept(x)
    print(colored("Correct coordinate for 'x'", "green"))
    
    y = input("\n- Enter 'y' position: ")
    number_y = is_number_tryexcept(y)
    while number_y is not True:
        print(colored("Invalid coordinate for 'y'", "yellow"))
        y = input("\n- Enter 'y' position: ")
        number_y = is_number_tryexcept(y)
    print(colored("Correct coordinate for 'y'", "green"))    
    
    rospy.wait_for_service('navigation')
    
    navigation = rospy.ServiceProxy('navigation', Coordinates)
    obtained_goal = navigation(float(x), float(y))
    
    if obtained_goal.results == True:
        clearConsole()
        pass
    else:
        clearConsole()
        pass
        
def second_option():
    """
    Function called in case if user entered second option in :func:`menu`. Provides users with a feature to self-control robot.
    Sends a *0* to :mod:`second_option` where it will run second option in case of **0**, while in case of **1** it will run third option.
    """
    print(colored("You choosed second option! Please, read the instructions for controlling the robot and go!", "green", attrs = ["bold", "underline"]))
    subprocess.Popen('roslaunch final_assignment second_option.launch', shell = True)
    rospy.wait_for_service('self_driving')
    self_driving = rospy.ServiceProxy('self_driving', TTS_server)
    self_driving(0)
 	
def third_option():
    """
    Function called in case if user entered third option in :func:`menu`. Provides users with a feature to self-control robot.
    Sends a **1** to :mod:`second_option` where it will run third option in case of **1**, while in case of **0** it will run second option.
    """
    print(colored("You choosed third option with obstacles avoidance feature! Please, read the instructions for controlling the robot and go!", "green", attrs = ["bold", "underline"]))
    subprocess.Popen('roslaunch final_assignment third_option.launch', shell = True)
    rospy.wait_for_service('self_driving')
    self_driving = rospy.ServiceProxy('self_driving', TTS_server)
    self_driving(1)

def fourth_option():
    """
    Function to exit from the program.
    """
    print("Good bye!")

def main():
    """
    Main function which manages entered string-inputs from :func:`menu` function. Equates ``typed_input`` with returned string-input from :func:`menu` function.
    
    * Checks whether ``typed_input`` is one of the following cases. Otherwise, prints that option is not in the list.
    
    * In case while loop is not true anymore, then exit from the program.
    """
    rospy.init_node('Menu')
    check_loop = 0
    while (check_loop == 0):
        typed_input = menu()

        if (typed_input == "1"):
            first_option()
        elif (typed_input == "2"):
            second_option()  
            os.system("rosnode kill self_driving >/dev/null")
            rospy.sleep(0.5)
        elif (typed_input == "3"):
            third_option()
            os.system("rosnode kill self_driving >/dev/null")
            os.system("rosnode kill adv_self_driving >/dev/null")
            rospy.sleep(0.5)
        elif (typed_input == "4"):
            fourth_option()
            os.system("rosnode kill first_option >/dev/null")
            check_loop = 1
        else:
            print(colored("That's not a valid option!", "yellow", attrs = ["blink"]))
            rospy.sleep(2)
    if check_loop == 1:
        sys.exit(0)
    

if __name__ == '__main__':
    """
    Executes main() function, which initializes ``Menu`` node.
    """
    main()
