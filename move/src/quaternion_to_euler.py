#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float32
def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    pub.publish(yaw)

rospy.init_node('quaternion_to_euler')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('/yaw_euler/odom',Float32, queue_size = 5)

r = rospy.Rate(1)

rospy.spin()