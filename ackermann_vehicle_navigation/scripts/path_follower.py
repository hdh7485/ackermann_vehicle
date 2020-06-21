#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import math

subscribed_path = Path()

def get_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def odom_callback(odom_msg):
    global subscribed_path
    nearest_distance = 10000
    nearest_index = 0
    for i in range(len(subscribed_path.poses)):
        distance = get_distance([odom_msg.pose.pose.position.x,
                                odom_msg.pose.pose.position.y], 
                                [subscribed_path.poses[i].pose.position.x,
                                subscribed_path.poses[i].pose.position.y])
        if distance < nearest_distance:
            nearest_distance = distance
            nearest_index = i
    rospy.loginfo(nearest_index)
    rospy.loginfo(nearest_distance)

def path_callback(path_msg):
    global subscribed_path
    subscribed_path = path_msg

if __name__ == '__main__':
    rospy.init_node('path_follower', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/path', Path, path_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass