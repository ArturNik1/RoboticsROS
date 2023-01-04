#!/usr/bin/env python3
import rospy
import actionlib
import time
from std_msgs.msg import String
from robotics_two.msg import Location, ObjectInfo, CustomPose
from robotics_two.srv import MoveTo, MoveToResponse, PickObject, PickObjectResponse, PlaceObject, PlaceObjectResponse, SenseObjects, SenseObjectsResponse, SensePose, SensePoseResponse
from geometry_msgs.msg import Point, Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates, ModelState
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from environment_functions import spawn_model, delete_model


import rospy


bag = "None"

def distance(x1, y1, x2, y2):
	dist = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** .5
	return dist


def movebase_client(x,y,w=1.0):
	#moves the robot collision free to a x,y,theta pose (must be valid/reachable in the map)
	client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
	client.wait_for_server()

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	goal.target_pose.pose.position.z = 0.0
	goal.target_pose.pose.orientation.w = w
	print("goal sent:")
	print(goal)
	client.send_goal(goal)
	wait = client.wait_for_result()
	if not wait:
		rospy.logerr("Action server not available!")
		rospy.signal_shutdown("Action server not available!")
		return False
	else:
		return bool(client.get_result())


def gps_location():
	# request a GPS like pose information from the Gazebo server
	rospy.loginfo("Requesting Global Robot Pose from Gazebo")
	model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
	me_pose = Pose()
	me_pose = model_state.pose[2]
	me_pose_angles = euler_from_quaternion([me_pose.orientation.x, me_pose.orientation.y, me_pose.orientation.z, me_pose.orientation.w])
	print('My pose is (x,y,theta): ')
	print(me_pose.position.x, me_pose.position.y, me_pose_angles[2])
	return me_pose.position.x, me_pose.position.y, me_pose_angles[2]


def find_objects():
	# request from Gazebo the global pose of all objects
	rospy.loginfo("Requesting Global Object Poses from Gazebo")
	model_state = rospy.wait_for_message("gazebo/model_states", ModelStates)
	number_of_objects = len(model_state.pose)  - 3 # ignore: [ground_plane, room1, turtlebot3_burger]    	  	   	
	print('I found ' +str(number_of_objects) +' Objects')
	print(model_state.name[3:])
	#print('They are at: ')
	#object_pose = []
	#for n in range(number_of_objects):
	#   object_pose.append(model_state.pose[3+n])
	#print(object_pose)
	return model_state.name[3:], model_state.pose[3:]


def pick_object(object_name, object_position):
	print('Trying to pick up: ' + object_name)
	me_pose = gps_location()
	object_x = object_position[0]
	object_y = object_position[1]	
	dist = distance(me_pose[0],me_pose[1],object_x,object_y)
	
	bag = rospy.get_param("bag")

	if dist <.35 and bag == "None":
		delete_model(object_name)
		time.sleep(1)
		spawn_model(name=object_name, spawn_location=[-1.0, -1.5, 1.0]) #put in knapsack
		rospy.set_param("bag", object_name)
		time.sleep(1)
		print('...successfully.')
		
	else: 
		print('...unsuccessfully. Need to be closer to the object to pick it')
		


def place_object(object_name, place_location):
	# delete selected object from bag and place it in gazebo
	me_pose = gps_location()
	dist2 = distance(me_pose[0], me_pose[1], place_location[0], place_location[1])
	bag = rospy.get_param("bag")
	isPicked = (bag == object_name)
	if not isPicked: 
		print('Object is not with me...')
		return False
	if dist2<.6:
		delete_model(object_name)
		rospy.sleep(1)
		spawn_model(name=object_name, spawn_location=place_location)
		rospy.set_param("bag", "None")
		print('Placed the object')
		return True
	else: 
		print('Need to be closer to the location to place the object (and NOT on it!)') 
		return False



def handle_move_to(req):
	x = req.location.x
	y = req.location.y
	result = movebase_client(x,y)
	pub_log = rospy.Publisher('/turtlebot3_log/add_to_log', String, queue_size=10)
	rospy.sleep(0.5)
	s = "rosservice call /turtlebot3_skills_server/move_to '[{}, {}, {}]'".format(x, y, req.location.z)
	pub_log.publish(s)
	return MoveToResponse(result)

def handle_pick_object(req):
	name = req.name
	x = req.location.x
	y = req.location.y
	pick_object(name, [x,y])
	pub_log = rospy.Publisher('/turtlebot3_log/add_to_log', String, queue_size=10)
	rospy.sleep(0.5)
	s = "rosservice call /turtlebot3_skills_server/pick_object {} '[{}, {}, {}]'".format(name, x, y, req.location.z)
	pub_log.publish(s)
	return PickObjectResponse()

def handle_place_object(req):
	name = req.name
	x = req.location.x
	y = req.location.y
	z = req.location.z
	result = place_object(name, [x,y,z])
	pub_log = rospy.Publisher('/turtlebot3_log/add_to_log', String, queue_size=10)
	rospy.sleep(0.5)
	s = "rosservice call /turtlebot3_skills_server/place_object {} '[{}, {}, {}]'".format(name, x, y, req.location.z)
	pub_log.publish(s)
	return PlaceObjectResponse(result)

def handle_sense_objects(req):
	names, poses = find_objects()
	objects = []
	for name, pose in zip(names, poses):
		object_info = ObjectInfo()
		object_info.name = name
		customPose = CustomPose()
		customPose.x = pose.position.x
		customPose.y = pose.position.y
		customPose.w = pose.orientation.w
		object_info.pose = customPose
		objects.append(object_info)
	
	pub_log = rospy.Publisher('/turtlebot3_log/add_to_log', String, queue_size=10)
	rospy.sleep(0.5)
	s = "rosservice call /turtlebot3_skills_server/sense_objects"
	pub_log.publish(s)

	return SenseObjectsResponse(objects)

def handle_sense_pose(req):
	x,y,w = gps_location()
	customPose = CustomPose()
	customPose.x = x
	customPose.y = y
	customPose.w = w
	pub_log = rospy.Publisher('/turtlebot3_log/add_to_log', String, queue_size=10)
	rospy.sleep(0.5)
	s = "rosservice call /turtlebot3_skills_server/sense_pose"
	pub_log.publish(s)
	return SensePoseResponse(customPose)

if __name__ == '__main__':
	# example script that picks one of the balls and places it
	# set up environment
	# initialize_environment() 
	

	rospy.init_node('turtlebot3_skills_server', anonymous=True)

	rospy.set_param("bag", "None")

	s1 = rospy.Service('/turtlebot3_skills_server/move_to', MoveTo, handle_move_to)
	s2 = rospy.Service('/turtlebot3_skills_server/sense_pose', SensePose, handle_sense_pose)
	s3 = rospy.Service('/turtlebot3_skills_server/pick_object', PickObject, handle_pick_object)
	s4 = rospy.Service('/turtlebot3_skills_server/place_object', PlaceObject, handle_place_object)
	s5 = rospy.Service('/turtlebot3_skills_server/sense_objects', SenseObjects, handle_sense_objects)
	#rospy.spin()
	rate = rospy.Rate(10)	

	#create_scene(True) # delete old env. if exists
	#create_scene(False) # set new environment
	
	# sense
	# my_pose = gps_location()
	# object_names, object_poses = find_objects()
	
	# # navigate to pickup
	# ball_placing_positions = ((1.711,0.236),(1.884,0.268), (1.763, 0.165), (1.837, 0.336), (1.8, 0.243))
	# for object_name in object_names:
	# 	if object_name == "blue_cube":
	# 		continue

	# 	pick_object_name = object_name
	# 	print('I navigate to '+ pick_object_name)
	# 	idx = object_names.index(pick_object_name) # find object pose by index
		
	# 	pick_object_position = [object_poses[idx].position.x, object_poses[idx].position.y]
	# 	last = int(pick_object_name[-1])
	# 	x_offset = 0
	# 	y_offset = 0
	# 	if last == 0:
	# 		x_offset = 0
	# 		y_offset = -0.15
	# 	elif last == 1:
	# 		x_offset = -0.15
	# 		y_offset = 0
	# 	elif last == 2:
	# 		x_offset = 0.15
	# 		y_offset = 0.15
	# 	elif last == 3:
	# 		x_offset = 0
	# 		y_offset = -0.15
	# 	elif last == 4:
	# 		x_offset = 0.15
	# 		y_offset = 0.15
	# 	pick_navigation_goal = [pick_object_position[0]+x_offset,pick_object_position[1]+y_offset]
	# 	result = movebase_client(pick_navigation_goal[0],pick_navigation_goal[1],1.0)
	# 	if result:
	# 		rospy.loginfo("Goal execution done!")

	# 	# pick
	# 	pick_object(pick_object_name,pick_object_position)

	# 	# navigate to dropoff 
	# 	idx = object_names.index('blue_cube') # find blue_cube pose by index
	# 	goal_object_position = [object_poses[idx].position.x, object_poses[idx].position.y]
	# 	place_navigation_goal = [2,0]
	# 	result = movebase_client(place_navigation_goal[0],place_navigation_goal[1],1.0)
	# 	if result:
	# 		rospy.loginfo("Goal execution done!")
			
	# 	# place
	# 	i = int(pick_object_name[-1])
	# 	success = place_object(pick_object_name, [ball_placing_positions[i][0],ball_placing_positions[i][1],0.05])
	# 	if success: 
	# 		print("Successfully moved the object")

	rospy.spin()

