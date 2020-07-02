#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import tf_conversions
import rospkg

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SetModelState

import math
import numpy as np
import glob

rospack = rospkg.RosPack()

vehicle_status = "Init"
subscribed_path = Path()
vehicle_odom = Odometry()
wp_index = 1
pre_beta = 0
pre_cte = 0
pre_time = 0
seq = 0

start_time = rospy.Time()
end_time = rospy.Time()

yaw_rate_limit = True
vx_limit = True

test_number = 0

def path_reader(directory):
    global subscribed_path
    
    posestamp_list = []
    seq = 0

    header_msg = Header()
    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = global_frame_id

    loaded_path = np.load(directory).get('path')

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

    header_msg.seq = seq
    header_msg.stamp = rospy.Time.now()
    header_msg.frame_id = global_frame_id

    path_msg.header = header_msg
    path_msg.poses = posestamp_list

    path_pub.publish(path_msg)
    seq += 1
    subscribed_path = path_msg
    # rospy.loginfo(subscribed_path)

def get_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def handle_vehicle_pose(msg, vehicle_name):
    global subscribed_path, wp_index, pre_beta, pre_cte, pre_time, seq, end_time, start_time
    global vehicle_odom, vehicle_status
    global test_number
    if vehicle_status == 'Generated':
        vehicle_index = msg.name.index(vehicle_name)
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = global_frame_id
        t.child_frame_id = vehicle_name
        #vehicle_odom.pose.pose = msg.pose[vehicle_index]
        t.transform.translation.x = msg.pose[vehicle_index].position.x + 1
        t.transform.translation.y = msg.pose[vehicle_index].position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose[vehicle_index].orientation

        br.sendTransform(t)
        vehicle_odom = Odometry()
        vehicle_odom.header.stamp = rospy.Time.now()
        vehicle_odom.header.frame_id = global_frame_id
        vehicle_odom.child_frame_id = vehicle_name
        #vehicle_odom.pose.pose = msg.pose[vehicle_index]
        vehicle_odom.pose.pose.position.x = msg.pose[vehicle_index].position.x + 1
        vehicle_odom.pose.pose.position.y = msg.pose[vehicle_index].position.y
        vehicle_odom.pose.pose.position.z = 0
        vehicle_odom.pose.pose.orientation = msg.pose[vehicle_index].orientation
        vehicle_odom.twist.twist = msg.twist[vehicle_index]
        odom_publisher.publish(vehicle_odom)

        # if wp_index == 1:
        if len(subscribed_path.poses) != 0:
            # Robot's current X, Y point
            current_point = np.array([vehicle_odom.pose.pose.position.x, vehicle_odom.pose.pose.position.y])

            # Robot's current heading
            quaternion = (vehicle_odom.pose.pose.orientation.x,
                          vehicle_odom.pose.pose.orientation.y,
                          vehicle_odom.pose.pose.orientation.z,
                          vehicle_odom.pose.pose.orientation.w
                         )

            euler = tf.transformations.euler_from_quaternion(quaternion)
            current_heading = euler[2]

            next_wp = np.array([subscribed_path.poses[wp_index].pose.position.x, subscribed_path.poses[wp_index].pose.position.y])
            next_point_distance = np.linalg.norm(next_wp - current_point)
            if next_point_distance < 0.4:
                wp_index = wp_index+1
                # rospy.loginfo("length:{} wp_index:{}".format(len(subscribed_path.poses), wp_index))
                if wp_index >= len(subscribed_path.poses):
                    test_number += 1
                    vehicle_status = 'Arrived'
                    wp_index = 1

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
            if yaw_rate_limit is True:
                yaw_rate = max(min(yaw_rate, 0.5), -0.5)

            # vx = 0.6
            vx = 0.7 - abs(beta*0.7 + cte*0.7 + heading_error*0.7)
            if vx_limit is True:
                vx = max(min(vx, 0.7), 0.1)
            # rospy.loginfo("wp_index:{} heading_error:{} heading:{} wp_heading:{} beta:{} cte:{} yaw_rate:{} vx:{}".format(subscribed_path.poses[wp_index-1], heading_error, current_heading, wp_heading, np.degrees(beta), cte, yaw_rate, vx))
            pre_beta = beta
            pre_cte = cte
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = vx
            cmd_vel_msg.angular.z = yaw_rate
            cmd_vel_pub.publish(cmd_vel_msg)

            seq = seq + 1

if __name__ == '__main__':
    rospy.init_node('path_follower', anonymous=True, disable_signals=True)

    package_path = rospack.get_path('ackermann_vehicle_navigation')
    test_directory = package_path + "/path/test_path.npz"
    
    path_directory = rospy.get_param('~tracking_path_directory', test_directory)
    global_frame_id = rospy.get_param('~global_frame_id', 'world')
    vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
    model_xml = rospy.get_param('/robot_description')
    log_file_path = rospy.get_param('~log_destination', package_path + '/log/{}_log.txt'.format(str(rospy.Time.now().to_sec())))

    path_pub = rospy.Publisher('/path', Path, queue_size=1)
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('/gazebo/model_states',
                     ModelStates,
                     handle_vehicle_pose,
                     vehicle_name,
                     queue_size=1)

    
    path_list = glob.glob(path_directory)
    rospy.loginfo(path_list)
    path_file = path_list[0]

    is_first_while = True
    pre_test_number = -1
    rospy.loginfo(log_file_path)
    while not rospy.is_shutdown():
        if test_number != pre_test_number:
            end_time = rospy.Time.now()
            if not is_first_while:
                elapsed_time = (end_time - start_time).to_sec()
                rospy.loginfo("{}".format(elapsed_time))
                f = open(log_file_path, 'a')
                f.write("{}, {}\n".format(path_list[test_number], elapsed_time))
                f.close()
                is_first_while = False
            rospy.wait_for_service('gazebo/set_model_state')
            init_pose = Pose(position=Point(-1,0,1.5), orientation=Quaternion(0, 0, 0.7071068, 0.7071068))
            model_init_state = ModelState(model_name='ackermann_vehicle', pose=init_pose,
                                          twist=Twist(), reference_frame="world")
            spawn_model_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
            spawn_model_prox(model_init_state)
            path_reader(path_list[test_number])
            rospy.loginfo("{} {}".format(test_number, path_list[test_number]))
            pre_test_number = test_number
            is_first_while = False
            wp_index = 1
            vehicle_status = "Generated"
            start_time = end_time
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
