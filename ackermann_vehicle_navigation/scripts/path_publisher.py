#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import tf
import rospkg

rospack = rospkg.RosPack()

def talker():
    rospy.init_node('path_publisher', anonymous=True)
    pub = rospy.Publisher('/path', Path, queue_size=1)

    package_path = rospack.get_path('ackermann_vehicle_navigation')
    test_directory = package_path + "/path/test_path.txt"
    path_directory = rospy.get_param('~tracking_path_directory', test_directory)
    global_frame_id = rospy.get_param('~global_frame_id', 'world')

    f = open(path_directory, 'r')

    rospy.loginfo(path_directory)
    rate = rospy.Rate(1) # 1hz
    
    lines = f.readlines()
    f.close()
    posestamp_list = []
    seq = 0

    header_msg = Header()
    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = global_frame_id

    for line in lines:
        path_msg = Path()
        posestamp_msg = PoseStamped()
        pose_msg = Pose()
        value = line.split()
        # rospy.loginfo(value)
        pose_msg.position.x = float(value[0])
        pose_msg.position.y = float(value[1])
        pose_msg.position.z = 0
        # pose_msg.orientation.x = 0
        # pose_msg.orientation.y = 0
        # pose_msg.orientation.z = float(value[5])
        # pose_msg.orientation.w = float(value[6])
        posestamp_msg.pose = pose_msg
        posestamp_msg.header = header_msg
        posestamp_list.append(posestamp_msg)
    # rospy.loginfo(posestamp_list)

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
