#!/usr/bin/env python
import os
import string
import sys
import rospy
from std_msgs.msg import String
from robotics_one.srv import PrintLog, PrintLogResponse

messages = []

def callback(data):
    messages.append(data.data)

    
def handle_print(req):
    log = ""
    for s in messages:
        log = log + s + "\n"
    print ("log: " + log)
    f = open(os.path.dirname(__file__) + '/../log.txt', 'w')
    f.write(log)
    f.close()
    return PrintLogResponse()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('printer', anonymous=True)

    s = rospy.Service('print_log_info/print', PrintLog, handle_print)
    rospy.Subscriber("received_requests", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
