#!/usr/bin/env python3arturnik
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
import rospy
from robotics_two.srv import CreateScene, CreateSceneResponse, GoalChecker, GoalCheckerResponse
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates, ModelState
import time
import random
from robotics_two.srv import SenseObjects, SenseObjectsRequest
from pathlib import Path


def initialize_environment():
    rospy.init_node('turtlebot3_ff', anonymous=True, log_level=rospy.WARN)

def delete_model(name):
    # delete model
    srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    req = DeleteModelRequest()
    req.model_name = name
    resp = srv(req)

def spawn_model(name, file_location='{}/.gazebo/models/objects/red_ball.sdf'.format(str(Path.home())), spawn_location=[0.0,0.0,1.0]):
    #rospy.init_node('spawn_model', log_level=rospy.INFO)
    pose = Pose()
    pose.position.x = spawn_location[0]
    pose.position.y = spawn_location[1]
    pose.position.z = spawn_location[2]
    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_client(model_name=name,
                       model_xml=open(file_location, 'r').read(),
                       robot_namespace='/stuff', initial_pose=pose, reference_frame='world')


def random_point(x1, x2, y1, y2):
    offset = 0.1
    return [random.uniform(min(x1, x2)+offset, max(x1, x2)-offset), random.uniform(min(y1, y2)+offset, max(y1, y2)-offset), 0.2]
    

def create_scene():  
    # costmap = Costmap2DROS()

    delete_model('blue_cube') 
    time.sleep(1)
    # ((1.1, -.47), (-0.72,0.3)) old 
    domains = [((3.838, 4.362), (0.34,0.75)), ((6.45, 5.06), (-0.9,0.06)), ((3, 4), (-1.37,-1.56)), ((2.33, 3.2), (1.21,0.37)), ((2.34, 2.67), (-1.15,-0.88))]
    ball_num = random.randint(3,5)
    spawn_locations = [random_point(domains[i][0][0],domains[i][0][1], domains[i][1][0], domains[i][1][1]) for i in range(ball_num)]    
    for n in range(len(spawn_locations)):
        delete_model('red_ball'+str(n)) 
        time.sleep(.5)
        spawn_model('red_ball'+str(n), '{}/.gazebo/models/objects/red_ball.sdf'.format(str(Path.home())), spawn_locations[n])
        # costmap.addObstacle(spawn_locations[n][0], spawn_locations[n][1], 0.0375)
    spawn_model('blue_cube', '{}/.gazebo/models/objects/blue_cube.sdf'.format(str(Path.home())), [1.707010,0.347668,1] )
    # costmap.updateCostmap()
      
def goal_checker():
    srv = rospy.ServiceProxy('/turtlebot3_skills_server/sense_objects', SenseObjects)
    req = SenseObjectsRequest()
    resp = srv(req)
    dom_x, dom_y = (1.7,2), (0,0.347)
    for obj in resp.objects:
        if obj.name == "blue_cube":
            continue

        #checks if all the balls are within the box range.
        if (obj.pose.x < dom_x[0] or obj.pose.x > dom_x[1])  or (obj.pose.y < dom_y[0] or  obj.pose.y > dom_y[1]):
            return False
    return True
        

    
def handle_create_scene(req):
    create_scene()
    return CreateSceneResponse()


def handle_goal_checker(req):
    return GoalCheckerResponse(goal_checker())


if __name__ == '__main__':
    rospy.init_node('environment_server', anonymous=True)
    s1 = rospy.Service('/environment_server/create_scene', CreateScene, handle_create_scene)
    s2 = rospy.Service('/environment_server/goal_checker', GoalChecker, handle_goal_checker)
    rate = rospy.Rate(10)
    rospy.spin()

    

    # initialize_environment()
    # create_scene()	

	

