#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates


def handle_vehicle_pose(msg, vehicle_name):
    vehicle_index = msg.name.index(vehicle_name)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = vehicle_name
    t.transform.translation.x = msg.pose[vehicle_index].position.x
    t.transform.translation.y = msg.pose[vehicle_index].position.y
    t.transform.translation.z = 0.0
    t.transform.rotation = msg.pose[vehicle_index].orientation

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tf_publisher')
    vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
    global_frame_id = rospy.get_param('~global_frame_id', 'world')
    rospy.loginfo(global_frame_id)
    rospy.Subscriber('/gazebo/model_states',
                     ModelStates,
                     handle_vehicle_pose,
                     vehicle_name)
    rospy.spin()