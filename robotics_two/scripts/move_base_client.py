import actionlib
import rospy;
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal;

if __name__ == '__main__':
    rospy.init_node('turtlebot3_ff', anonymous=True, log_level=rospy.WARN)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 3.53
    goal.target_pose.pose.position.y = -1.4
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print(client.get_result())