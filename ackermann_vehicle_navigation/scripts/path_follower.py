#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import math
import numpy as np

subscribed_path = Path()
wp_index = 0
pre_beta = 0
pre_cte = 0
pre_time = 0
seq = 0

def get_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def odom_callback(odom_msg):
    global subscribed_path, wp_index, pre_beta, pre_cte, pre_time, seq
    # nearest_distance = 10000
    # nearest_index = 0
    # for i in range(len(subscribed_path.poses)):
        # distance = get_distance([odom_msg.pose.pose.position.x,
                                # odom_msg.pose.pose.position.y], 
                                # [subscribed_path.poses[i].pose.position.x,
                                # subscribed_path.poses[i].pose.position.y])
        # if distance < nearest_distance:
            # nearest_distance = distance
            # nearest_index = i

    current_point = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
    next_wp = np.array([subscribed_path.poses[wp_index+1].pose.position.x, subscribed_path.poses[wp_index+1].pose.position.y])
    next_point_distance = np.linalg.norm(next_wp - current_point)
    if next_point_distance < 0.2:
        wp_index = wp_index+1

    # rospy.loginfo(nearest_index)
    # nearest_front_point = np.array([subscribed_path.poses[nearest_index+1].pose.position.x, subscribed_path.poses[nearest_index+1].pose.position.y])
    # nearest_rear_point = np.array([subscribed_path.poses[nearest_index].pose.position.x, subscribed_path.poses[nearest_index].pose.position.y])
    current_wp = np.array([subscribed_path.poses[wp_index].pose.position.x, subscribed_path.poses[wp_index].pose.position.y])
    next_wp = np.array([subscribed_path.poses[wp_index+1].pose.position.x, subscribed_path.poses[wp_index+1].pose.position.y])
    
    ba = current_point - next_wp
    bc = current_wp - next_wp
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))

    position = 1
    if np.cross(ba, bc) > 0:
        position = -1

    beta = position*np.arccos(cosine_angle)
    cte = position*np.linalg.norm(np.cross(bc, -ba))/np.linalg.norm(bc)
    
    if seq is 0:
        pre_time = rospy.Time.now() - rospy.Duration(0.001)
    current_time = rospy.Time.now()
    dt = (current_time - pre_time).to_sec()
    # rospy.loginfo(dt)
    beta_dot = (beta - pre_beta) / dt
    cte_dot = (cte - pre_cte) / dt
    pre_tiem = current_time
    
    # yaw_rate = beta*0.6 + beta_dot*3.5 + cte*0.002 + cte_dot*0.05
    yaw_rate = beta*1.0 + cte*1.00 + beta_dot*1.0 + cte_dot*0.05
    vx = 0.2
    rospy.loginfo("wp_index:{} beta:{} cte:{} yaw_rate:{} vx:{}".format(wp_index, np.degrees(beta), cte, yaw_rate, vx))
    pre_beta = beta
    pre_cte = cte
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = vx
    cmd_vel_msg.angular.z = yaw_rate
    cmd_vel_pub.publish(cmd_vel_msg)

    seq = seq + 1

def path_callback(path_msg):
    global subscribed_path
    subscribed_path = path_msg

if __name__ == '__main__':
    rospy.init_node('path_follower', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/path', Path, path_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=1)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass