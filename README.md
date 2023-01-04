# RoboticsROS

You can see the final version in the mp4 file.
 
How to run the code:

1. run the commands: 
	'roscore'
	'rosrun turtlesim turtlesim_node'
   on two different terminals.

2. run 'rosrun robotics_one draw_letter_server.py' to start the server.
3. run 'rosrun robotics_one print_log_info.py' to save log history.
4. run 'rosrun robotics_one draw_letter_client.py '[x, y]' "a/o/z" '[r, g, b]' to draw a letter.
5. if you want to print log history:
	run 'rosservice call /print_log_info/print'


To run the script (part 3) run 'python3 assignment_one.py' in the correct folder.

The log.txt file will be saved int the main directory "robotics_one"
