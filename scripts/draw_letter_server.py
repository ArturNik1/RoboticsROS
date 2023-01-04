#!/usr/bin/env python
#https://people.eng.unimelb.edu.au/pbeuchat/asclinic/software/ros_define_and_use_custom_message_types.html
from __future__ import print_function

import time
from robotics_one.srv import DrawLetter, DrawLetterResponse
from robotics_one.msg import Location, Color
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from std_msgs.msg import String


import rospy


turtle = Pose()
vel = Twist()

def handle_draw_letter(req):
    user_input = ""
    color = req.color
    location = req.location
    letter = req.letter


    pub_received_requests = rospy.Publisher('received_requests', String, queue_size=10)
    rospy.sleep(0.5)
    user_input = """rosrun robotics_one draw_letter_client '[{},{}]' "{let}" '[{r}, {g}, {b}]''""".format(location.x, location.y, let =letter, r = color.r, g = color.g, b = color.b)
    pub_received_requests.publish(user_input)

    teleport_turtle(location,color)

    pub_cmd_vel = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    if (letter == "z"):
        draw_z(pub_cmd_vel)
    elif (letter == "a"):
        draw_a(pub_cmd_vel, color)
    elif (letter == "o"):
        draw_o(pub_cmd_vel)

    return DrawLetterResponse(1)

def go_to(x,y,pub,vel):
    rospy.sleep(0.3)
    vel.linear.x = x
    vel.linear.y = y
    pub.publish(vel)
    rospy.sleep(1)

def draw_z(pub):
    vel = Twist()
    pub.publish(vel)
    go_to(2.0,0,pub,vel)
    go_to(-2.0,-2.0,pub,vel)
    go_to(2,0,pub,vel)

def draw_a(pub, color):
    vel = Twist()
    pub.publish(vel)
    go_to(1,2.0,pub,vel)
    go_to(1.0,-2.0,pub,vel)
    move_turtle(-0.4,0.8,color,pub)
    go_to(-1.2,0,pub,vel)
    
def draw_o(pub):
    vel = Twist()
    pub.publish(vel)
    vel.linear.x = 1.0
    vel.angular.z = 1.0
    now = rospy.Time.now()
    rate = rospy.Rate(10)

    while rospy.Time.now() < now + rospy.Duration.from_sec(6):
        pub.publish(vel)
        rate.sleep()
    
def teleport_turtle(location, color):
    rospy.wait_for_service('turtle1/set_pen')
    try:
        set_pen_resp = set_pen(color.r, color.g, color.b, 2, 1)
    except rospy.ServiceException as e:
        print("Service call failes: %s", e)


    rospy.wait_for_service('turtle1/teleport_absolute')
    try:
        teleportResp = teleport_absolute(location.x,location.y,0)
    except rospy.ServiceException as e:
        print("Service call failes: %s", e)
        

    rospy.wait_for_service('turtle1/set_pen')
    try:
        set_pen_resp = set_pen(color.r, color.g, color.b, 2, 0)
    except rospy.ServiceException as e:
        print("Service call failes: %s", e)


def move_turtle(x, y, color, pub):
    rospy.wait_for_service('turtle1/set_pen')
    try:
        set_pen_resp = set_pen(color.r, color.g, color.b, 2, 1)
    except rospy.ServiceException as e:
        print("Service call failes: %s", e)

    vel = Twist()
    pub.publish(vel)
    rospy.sleep(0.3)
    vel.linear.x = x
    vel.linear.y = y
    pub.publish(vel)
    rospy.sleep(1)
        
    rospy.wait_for_service('turtle1/set_pen')
    try:
        set_pen_resp = set_pen(color.r, color.g, color.b, 2, 0)
    except rospy.ServiceException as e:
        print("Service call failes: %s", e)



def update_pose(data):
    turtle = data

def draw_letter_server():
    rospy.init_node('draw_letter_server', anonymous=True)
    s = rospy.Service('draw_letter', DrawLetter, handle_draw_letter)
    rospy.Subscriber('turtle1/pose', Pose, update_pose)
    rospy.spin()

if __name__ == "__main__":
    # get services
    set_pen = rospy.ServiceProxy('turtle1/set_pen', SetPen)
    teleport_absolute = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)

    # init service server
    draw_letter_server()

    # set publishers
    pub_cmd_vel = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
