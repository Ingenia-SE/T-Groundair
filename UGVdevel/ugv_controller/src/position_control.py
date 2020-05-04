#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import Odometry

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = tf.transformations.euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("pos_controller")

sub = rospy.Subscriber("/sim_p3at/odom", Odometry, newOdom)
pub = rospy.Publisher("/sim_p3at/cmd_vel", Twist, queue_size=1)

speed = Twist()
r = rospy.Rate(4)
goal = Point()
goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y
    
    angle_to_goal = math.atan2(inc_y, inc_x)

    if abs(angle_to_goal - theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3
    else:
        speed.linear.x = 0.5
        speed.angular.z = 0.0

    pub.publish(speed)
    r.sleep()