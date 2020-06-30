#!/usr/bin/env python  
import rospy

import tf_conversions

import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

def handle_vehicle_pose(msg, vehicle_name):
    vehicle_index = msg.name.index(vehicle_name)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = global_frame_id
    t.child_frame_id = vehicle_name
    #odom_msg.pose.pose = msg.pose[vehicle_index]
    t.transform.translation.x = msg.pose[vehicle_index].position.x + 1
    t.transform.translation.y = msg.pose[vehicle_index].position.y
    t.transform.translation.z = 0.0
    t.transform.rotation = msg.pose[vehicle_index].orientation

    br.sendTransform(t)
    odom_msg = Odometry()
    odom_msg.header.stamp = rospy.Time.now()
    odom_msg.header.frame_id = global_frame_id
    odom_msg.child_frame_id = vehicle_name
    #odom_msg.pose.pose = msg.pose[vehicle_index]
    odom_msg.pose.pose.position.x = msg.pose[vehicle_index].position.x + 1
    odom_msg.pose.pose.position.y = msg.pose[vehicle_index].position.y
    odom_msg.pose.pose.position.z = 0
    odom_msg.pose.pose.orientation = msg.pose[vehicle_index].orientation
    odom_msg.twist.twist = msg.twist[vehicle_index]
    odom_publisher.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('tf_odom_publisher')
    vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
    global_frame_id = rospy.get_param('~global_frame_id', 'world')
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
    rospy.Subscriber('/gazebo/model_states',
                     ModelStates,
                     handle_vehicle_pose,
                     vehicle_name)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass