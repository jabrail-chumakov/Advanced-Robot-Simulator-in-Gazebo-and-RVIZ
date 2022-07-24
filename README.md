Advanced-Robot-Simulator-in-Gazebo-and-RVIZ
================================

This is a advanced robot simulator in ROS, which has four available options. It was done as a part of Research Track I course and written in Python.

Documentation
----------------------

Documentation generated with Sphinx can be found on the following [link](https://jabrail-chumakov.github.io/Advanced-Robot-Simulator-in-Gazebo-and-RVIZ/).

Installing
----------------------

To run this program you should first git clone it and place into `final_assigment` folder in your catkin workspace. Besides that you need:
- Download Slam gmapping from this [repository](https://github.com/CarmineD8/slam_gmapping)
- Ros Navigation Stack, by entering following command in terminal: `apt-get install ros-<your_ros_distro>-navigation`
- Teleop Twist Keyboard, by entering following command in terminal: `sudo apt-get install ros-<your_ros_distro>-teleop-twist-keyboard`
- Install following python libraries: `pynput`, `termcolor`, `tabulate`, `termios` and `inputimeout`.

Exercise
-----------------------------

In order to run the program, you should enter the commands below in the following order:

```bash
$ roslaunch final_assignment simulation_gmapping.launch
$ roslaunch final_assignment move_base.launch
$ roslaunch final_assignment main_menu.launch
```
The first script should run Gazebo and RVIZ in the new windows. The second script will run `move_base` launch file, while the last one should open the program itself.

In order to run the same program via **Jupiter**, run the following commands:

```bash
$ roslaunch final_assignment simulation_gmapping.launch
$ roslaunch final_assignment move_base.launch
$ roslaunch final_assignment jupyter_run.launch
```
After that you should run Jupyter notebook and run inside `jupyter_run.ipynb` file all cells.

Task objectives
---------
### Requirements ###

Develop a software architecture for the control of the robot in the environment. The software will rely on the `move_base` and `gmapping` packages for localizing the robot and planning the motion.

The architecture is able to get the user request, and let the robot execute one of the following behaviors (depending on the user's input):
- First option: autonomously reach x,y coordinate inserted by the user
- Second option: let the user drive the robot with the keyboard
- Third option: let the user drive the robot assisting them to avoid collisions
- Fourth option: close program

Working principle of proposed solution
-----------------------------

The proposed solution is simple and efficient and works almost perfectly. The solution consists of several cases when the robot needs to complete a particular command.

### Main menu - Interface ###

- The user is able to enter one of the desired commands described above. In case the input is `character` or `number` that is not in the list, it will display that this command is not valid. 

### Main menu - First option ###

- In case the user entered `1` in the input window, the program will ask in which coordinate robot should move for both `x` and `y`. In case of incorrect input, the program will inform the user that entered input is incorrect. If everything will be OK, then the robot will start moving to the target position. 
- One of the important features of this program is that the user can cancel the movement of the robot by pressing the `ESC` button. After that user will be returned to the main menu again, where he/she can choose desired option again. In case if the input for cancellation was incorrect, the system will inform the user. 
- The timeout for completing movement is 60 seconds, which means that after that time, the system will show the user state of the robot. In case of success, it will return `True`, otherwise, it will return `False`. After that, the system will inform user that main menu will appear again in 5 seconds.
- It should be noted that you can monitor the movement of your robot in `Gazebo` or `RVIZ`. From the simulator, you can observe that point in the center is coordinates for `(x, y)` = `(0, 0)`. And after observing that it's a room it can be concluded that the robot can't move to certain positions which are outside the room. That means if you will enter `(x, y)` = `(5, 5)`, then the robot will just try to find this location for 60 seconds and finally return `False` indicating also the current state of the robot.

### Main menu - Second option ###

- In case the user entered `2` in the input window, the program will open `second_option.py` script, where it will choose 1st option between two of them. First option is the robot control by the user without obstactles avoidance feature turned on. By pressing corresponding buttons, user can move the robot in all direction and also can collide into the wall.
- In case the user wants to stop operation, it's possible to press `Ctrl + C` command and after 5 seconds, main menu will appear again, where you can choose desired option again.

### Main menu - Third option ###

- In case the user entered `3` in the input window, the program will open both `second_option.py` and `third_option.py` scripts. From 1st script it will choose second option, which will run `third_option_teleop.launch`. In this operational mode, user can also control the robot, but now, with obstacles avoidance feature turned on. 
- If the user will go too close to the obstacle, which is approaching from right side, then it will not be possible to turn right anymore, until safe distance. 
- If the user will go too close to the obstacle, which is approaching from left side, then it will not be possible to left anymore, until safe distance.
- If the user will go too close to the obstacle, which is in front of the robot, then it will not be possible to move further anymore, until safe distance.
- In case the user wants to stop operation, it's possible to press `Ctrl + C` command and after 5 seconds, main menu will appear again, where you can choose desired option again.

### Main menu - Fourth option ###

- In case the user entered `4` in the input window, the program will close automatically. 

Working principle via Jupyter
-----------------------------
The program works in the same way as in the original project. In the first option, you just navigate to the desired position, while in the second and third you can move the robot by yourself. In order to enable obstacle collision, you should check the appropriate box inside Jupiter notebook.

Video demonstration
-----------------------------
Below you can watch a demonstration of this assignment (click on image):

[<img src="https://user-images.githubusercontent.com/67557966/180658532-2e5b7581-aad9-48d4-bf04-7a5138c49515.jpg" width="60%">](https://www.youtube.com/watch?v=Rp8nScg7rpc))

Flowchart
-----------------------------
![flowcharts](https://user-images.githubusercontent.com/67557966/153733475-f29e3aa0-76fb-40a2-9106-d8fb695bdbdc.png)


Possible improvements
-----------------------------

Despite the fact that this script works pretty well, there are still some moments that could be improved in future.

- For the first option, after entering desired coordinates, it will be possible to cancel operation by pressing `ESC` button. However, you may notice that after pressing `Arrow keys` the program will decide to return to the main menu either. That may be due to the usage of `\x1b` which is represents `ESC`. It can be fixed by changing `ESC` on any other character button or by blocking `Arrow keys` from the input. 
- For the first option, there is exist timeout in order to complete the movement. However, let's assume that next point is close to the current robot position. Then it will be sad for user to wait for additional `~50` seconds. In order to solve this problem it possible to add additional loop, which will check whether robot reached final position or not. If this occured before timeout, then it will be possible to display current goal state right after robot will reach desired point. 
- It also possible to implement `timeout on input` feature. It means that timeout can be choosed by the user in the beginning, after user choosed 1st option. 
- Also, in case if the new coordinates are the same as the previous ones, program should inform user that robot already in this position.
