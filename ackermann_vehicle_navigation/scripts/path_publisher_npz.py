#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf
import rospkg
import numpy as np

rospack = rospkg.RosPack()

def talker():
    rospy.init_node('path_publisher', anonymous=True)
    pub = rospy.Publisher('/path', Path, queue_size=1)

    package_path = rospack.get_path('ackermann_vehicle_navigation')
    test_directory = package_path + "/path/test_path.npz"
    
    path_directory = rospy.get_param('~tracking_path_directory', test_directory)
    global_frame_id = rospy.get_param('~global_frame_id', 'world')

    rospy.loginfo(path_directory)
    rate = rospy.Rate(1) # 1hz
    
    posestamp_list = []
    seq = 0

    header_msg = Header()
    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = global_frame_id

    loaded_path = np.load(path_directory).get('path')

    for point in loaded_path:
        path_msg = Path()
        posestamp_msg = PoseStamped()
        pose_msg = Pose()
        pose_msg.position.x = -point[1]
        pose_msg.position.y = point[0]
        pose_msg.position.z = 0
        posestamp_msg.pose = pose_msg
        posestamp_msg.header = header_msg
        posestamp_list.append(posestamp_msg)

    while not rospy.is_shutdown():
        header_msg.seq = seq
        header_msg.stamp = rospy.Time.now()
        header_msg.frame_id = global_frame_id

        path_msg.header = header_msg
        path_msg.poses = posestamp_list

        pub.publish(path_msg)
        seq += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass