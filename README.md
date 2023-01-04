# RoboticsROS
 
You can see the task final version in the mp4 files.

How to run the code:

# map.yaml and map.pgm should be in your home folder.
# run the bash script setfiles.sh. To run the setfiles.sh bash script, use the following command in the terminal: ./setfiles.sh. This command will copy all necessary files and folders to your workspace.
# To find the log file with the command history, navigate to the scripts folder in the robotics_two package. It should be located there.
# To run the assignment, run these commands in the terminal:

	1. roslaunch robotics_two robotics2.launch
	2. in differet terminal, run cd ~/catkin_ws/src/robotics_two/scripts && python3 turtlebot3_functions.py
	3. rosservice call /environment_server/create_scene
	4. cd ~/catkin_ws/src/robotics_two/scripts && python3 collect_all.py
	5. rosservice call /environment_server/goal_checker
	6. rosservice call /turtlebot3_log/print_save_log
	
	
=====================

if the setfiles.sh file won't work, you can do it manually:
# Copy the map.pgm and map.yaml files into the home directory
# Copy the models folder into the .gazebo folder in the home directory
# Put the files in the worlds folder into the turtlebot3_simulations/turtlebot3_gazebo/worlds directory
# Put the files in the worlds folder into the turtlebot3_simulations/turtlebot3_gazebo/launch directory
# Put the robotics_two folder in the catkin_ws/src directory
# Go to the catkin_ws directory and perform catkin_make
