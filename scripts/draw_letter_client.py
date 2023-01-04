#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from robotics_one.srv import DrawLetter
from robotics_one.msg import Location, Color

def draw_letter_client(location, letter, color):
    rospy.wait_for_service('draw_letter')
    try:
        draw_letter = rospy.ServiceProxy('draw_letter', DrawLetter)
        resp1 = draw_letter(location, letter, color)
        return resp1.flag
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 4:
        location = Location()
        locationArray = [float(i) for i in sys.argv[1][1:len(sys.argv[1])-1].split(',')]
        location.x = locationArray[0]
        location.y = locationArray[1]
        letter = sys.argv[2]
        color = Color()
        color_array = [int(i) for i in sys.argv[3][1:len(sys.argv[3])-1].split(',')]
        color.r = color_array[0]
        color.g = color_array[1]
        color.b = color_array[2]
    else:
        print(usage())
        sys.exit(1)
    draw_letter_client(location, letter, color)