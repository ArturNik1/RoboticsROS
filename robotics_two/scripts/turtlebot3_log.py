import os
from std_msgs.msg import String
import rospy
from robotics_two.srv import PrintSaveLog, PrintSaveLogResponse

log =[]

def handle_print_save_log(req):
    message = ""
    for s in log:
        message = message + s + '\n'
    print("log: " + message)
    f = open('log.txt', 'w')
    f.write(message)
    f.close()
    return PrintSaveLogResponse()

def handle_add_to_log(data):
    log.append(data.data)


if __name__ == '__main__':
    rospy.init_node('turtlebot3_log', anonymous=True)
    s1 = rospy.Service('/turtlebot3_log/print_save_log', PrintSaveLog, handle_print_save_log)
    rospy.Subscriber('/turtlebot3_log/add_to_log', String, handle_add_to_log)
    rate = rospy.Rate(10)
    rospy.spin()