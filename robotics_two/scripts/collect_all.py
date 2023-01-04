from robotics_two.srv import PlaceObject, PlaceObjectRequest, SenseObjects, SenseObjectsRequest, SensePose, SensePoseRequest, MoveTo, MoveToRequest, PickObject, PickObjectRequest
import rospy
from robotics_two.msg import Location, ObjectInfo, CustomPose


if __name__ == '__main__':
    # sense
    # my_pose = gps_location()
    srv = rospy.ServiceProxy('/turtlebot3_skills_server/sense_pose', SensePose)
    rospy.wait_for_service('/turtlebot3_skills_server/sense_pose')
    req_sense_pose = SensePoseRequest()
    my_pose = srv(req_sense_pose).pose

    # object_names, object_poses = find_objects()
    srv = rospy.ServiceProxy('/turtlebot3_skills_server/sense_objects', SenseObjects)
    rospy.wait_for_service('/turtlebot3_skills_server/sense_objects')
    req_find_objects = SenseObjectsRequest()
    resp = srv(req_find_objects)
    objects = resp.objects
    object_names = [obj.name for obj in objects]
    object_poses = [obj.pose for obj in objects]






    # navigate to pickup
    ball_placing_positions = ((1.711,0.236),(1.884,0.268), (1.763, 0.165), (1.837, 0.336), (1.8, 0.243))
    for object_name in object_names:
        if object_name == "blue_cube":
            continue

        pick_object_name = object_name
        print('I navigate to '+ pick_object_name)
        idx = object_names.index(pick_object_name) # find object pose by index
        
        pick_object_position = [object_poses[idx].x, object_poses[idx].y]
        last = int(pick_object_name[-1])
        x_offset = 0
        y_offset = 0
        if last == 0:
            x_offset = 0
            y_offset = -0.15
        elif last == 1:
            x_offset = -0.15
            y_offset = 0
        elif last == 2:
            x_offset = 0.15
            y_offset = 0.15
        elif last == 3:
            x_offset = 0
            y_offset = -0.15
        elif last == 4:
            x_offset = 0.15
            y_offset = 0.15

        pick_navigation_goal = [pick_object_position[0]+x_offset,pick_object_position[1]+y_offset]
        location = Location()
        location.x = pick_navigation_goal[0]
        location.y = pick_navigation_goal[1]
        location.z = 1.0
        srv = rospy.ServiceProxy('/turtlebot3_skills_server/move_to', MoveTo)
        rospy.wait_for_service('/turtlebot3_skills_server/move_to')
        req_move_to = MoveToRequest(location)
        result = srv(req_move_to).result

        #result = movebase_client(pick_navigation_goal[0],pick_navigation_goal[1],1.0)

        if result:
            rospy.loginfo("Goal execution done!")

        # pick
        srv = rospy.ServiceProxy('/turtlebot3_skills_server/pick_object', PickObject)
        rospy.wait_for_service('/turtlebot3_skills_server/pick_object')
        location = Location()
        location.x = pick_object_position[0]
        location.y = pick_object_position[1]
        location.z = 1.0
        req_pick_object = PickObjectRequest(pick_object_name, location)
        resp = srv(req_pick_object)
        # pick_object(pick_object_name,pick_object_position)

        # navigate to dropoff 
        idx = object_names.index('blue_cube') # find blue_cube pose by index
        goal_object_position = [object_poses[idx].x, object_poses[idx].y]
        place_navigation_goal = [2,0]


        location = Location()
        location.x = place_navigation_goal[0]
        location.y = place_navigation_goal[1]
        location.z = 1.0
        srv = rospy.ServiceProxy('/turtlebot3_skills_server/move_to', MoveTo)
        rospy.wait_for_service('/turtlebot3_skills_server/move_to')
        req_move_to = MoveToRequest(location)
        result = srv(req_move_to).result
        
        #result = movebase_client(place_navigation_goal[0],place_navigation_goal[1],1.0)

        if result:
            rospy.loginfo("Goal execution done!")
            
        # place
        i = int(pick_object_name[-1])
        srv = rospy.ServiceProxy('/turtlebot3_skills_server/place_object', PlaceObject)
        rospy.wait_for_service('/turtlebot3_skills_server/place_object')
        location = Location()
        location.x = ball_placing_positions[i][0]
        location.y = ball_placing_positions[i][1]
        location.z = 0.05
        req_place_object = PlaceObjectRequest(pick_object_name, location)
        resp = srv(req_place_object)
        # success = place_object(pick_object_name, [ball_placing_positions[i][0],ball_placing_positions[i][1],0.05])
        if resp.result: 
            print("Successfully moved the object")