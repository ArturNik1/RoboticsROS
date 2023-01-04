
# Copy the map.pgm and map.yaml files into the home directory
cp map.pgm map.yaml ~

# Copy the models folder into the .gazebo folder in the home directory
cp -r models ~/.gazebo

# Put the files in the worlds folder into the turtlebot3_simulations/turtlebot3_gazebo/worlds directory
cp worlds/* ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds

# Put the files in the worlds folder into the turtlebot3_simulations/turtlebot3_gazebo/launch directory
cp launch/* ~/catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch

# Put the robotics_two folder in the catkin_ws/src directory
cp -r robotics_two ~/catkin_ws/src

# Go to the catkin_ws directory and perform catkin_make
cd ~/catkin_ws
catkin_make
