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
import tf

subscribed_path = Path()
wp_index = 1
pre_beta = 0
pre_cte = 0
pre_time = 0
seq = 0

start_time = rospy.Time()
end_time = rospy.Time()

yaw_rate_limit = True
vx_limit = True

def get_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def odom_callback(odom_msg):
    global subscribed_path, wp_index, pre_beta, pre_cte, pre_time, seq, end_time, start_time
    if len(subscribed_path.poses) != 0:
        # Robot's current X, Y point
        current_point = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])

        # Robot's current heading
        quaternion = (odom_msg.pose.pose.orientation.x,
                      odom_msg.pose.pose.orientation.y,
                      odom_msg.pose.pose.orientation.z,
                      odom_msg.pose.pose.orientation.w
                     )

        euler = tf.transformations.euler_from_quaternion(quaternion)
        current_heading = euler[2]

        next_wp = np.array([subscribed_path.poses[wp_index].pose.position.x, subscribed_path.poses[wp_index].pose.position.y])
        next_point_distance = np.linalg.norm(next_wp - current_point)
        if next_point_distance < 0.4:
            wp_index = wp_index+1
            # rospy.loginfo("length:{} wp_index:{}".format(len(subscribed_path.poses), wp_index))
            if wp_index >= len(subscribed_path.poses):
                end_time = rospy.Time.now()
                rospy.loginfo((end_time - start_time).to_sec())
                rospy.signal_shutdown("Path following finish")

        current_wp = np.array([subscribed_path.poses[wp_index-1].pose.position.x, subscribed_path.poses[wp_index-1].pose.position.y])
        next_wp = np.array([subscribed_path.poses[wp_index].pose.position.x, subscribed_path.poses[wp_index].pose.position.y])
        wp_heading = math.atan2(next_wp[1] - current_wp[1], next_wp[0] - current_wp[0])

        heading_error = np.degrees(wp_heading) - np.degrees(current_heading)
        heading_error = np.radians((heading_error + 180) % 360 - 180)
    
        ba = current_point - next_wp
        bc = current_wp - next_wp
        cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))

        position = 1
        if np.cross(ba, bc) > 0:
            position = -1

        beta = position*np.arccos(cosine_angle)
        cte = position*np.linalg.norm(np.cross(bc, -ba))/np.linalg.norm(bc)
    
        current_time = rospy.Time.now()
        if seq is 0:
            start_time = current_time
            pre_time = rospy.Time.now() - rospy.Duration(0.001)

        dt = (current_time - pre_time).to_sec()
        # rospy.loginfo(dt)
        beta_dot = (beta - pre_beta) / dt
        cte_dot = (cte - pre_cte) / dt
        pre_tiem = current_time
    
        # yaw_rate = beta*1.0 + cte*1.00 + beta_dot*1.0 + cte_dot*0.05
        yaw_rate = beta*0.7 + cte*0.5 + heading_error*1.0
        # yaw_rate = beta*0.8 + cte*0.6
        if yaw_rate_limit is True:
            yaw_rate = max(min(yaw_rate, 0.5), -0.5)

        # vx = 0.6
        vx = 0.7 - abs(beta*0.7 + cte*0.7 + heading_error*0.7)
        if vx_limit is True:
            vx = max(min(vx, 0.7), 0.1)
        rospy.loginfo(vx)
        # rospy.loginfo("wp_index:{} heading_error:{} heading:{} wp_heading:{} beta:{} cte:{} yaw_rate:{} vx:{}".format(subscribed_path.poses[wp_index-1], heading_error, current_heading, wp_heading, np.degrees(beta), cte, yaw_rate, vx))
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
    rospy.init_node('path_follower', anonymous=True, disable_signals=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/path', Path, path_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=1)

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
