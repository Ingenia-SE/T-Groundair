#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import Odometry
from ugv_controller.msg import Order

x = 0.0
y = 0.0
theta = 0.0
idle = True
targetX = 0.0
targetY = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = tf.transformations.euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def parseOrder(msg):
    global targetX
    global targetY

    idle = False
    targetX = msg.point.x
    targetY = msg.point.y    
    
rospy.init_node("ugv_server")

#sub = rospy.Subscriber("/pos_controller/orders", Order, parseOrder)
pub = rospy.Publisher("/pos_controller/orders", Order, queue_size = 5)

#speed = Twist()
r = rospy.Rate(4)
#goal = Point()
#goal.x = 5
#goal.y = 5

while not rospy.is_shutdown():
    #inc_x = goal.x - x
    #inc_y = goal.y - y
    
    #angle_to_goal = math.atan2(inc_y, inc_x)
    #distance_to_goal = math.sqrt(inc_x*inc_x+inc_y*inc_y)

    #if abs(angle_to_goal - theta) > 0.1:
    #    speed.linear.x = 0.0
    #    speed.angular.z = 0.3
    #elif (distance_to_goal > 0.2):
    #    speed.linear.x = 0.5
    #    speed.angular.z = 0.0
    #else:
    #    speed.linear.x = 0.0
    #    speed.angular.z = 0.0

    #pub.publish(speed)
    #r.sleep()
    #rospy.sleep(10)
    #if idle == True:
   tst = Order()
   tst.order = "move"
   tst.point.x = 5.0
   tst.point.y = 5.0
   tst.point.z = 0.0
   pub.publish(tst)
   print("Publishing")
   idle = False
   r.sleep()
