#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import Odometry
from ugv_controller.msg import Order
from std_msgs.msg import Int16

# Store the waypoints we receive to send them to pos_controller
xTarget = 0
yTarget = 0
# Store a flag to check if we've received a message
orderReceived = False
# Store the status of the ugv
ugvStatus = 0

def parseWaypointSub(msg):
    """Receive and store orders received from server"""
    global xTarget, yTarget, orderReceived
    xTarget = msg.x
    yTarget = msg.y
    orderReceived = True

def sendWaypoint(publisher):
    """Sends orders to the position controller"""
    global xTarget, yTarget, orderReceived
    if orderReceived:
        orderReceived = False
        order = Order()
        order.order = "move"
        order.point.x = xTarget
        order.point.y = yTarget
        order.point.z = 0.614668
        publisher.publish(order)

def parseStatusSub(msg):
    """Receive and store status from ugv controller"""
    global ugvStatus
    ugvStatus = msg.data

def sendStatus(publisher):
    """Send ugv status to server"""
    global ugvStatus
    status = Int16()
    status.data = ugvStatus
    publisher.publish(status)

# Initialize ROS node
rospy.init_node("ugv_server")

# Create a subscriber to receive waypoints from server
waypointSub = rospy.Subscriber("/ugv_waypoints", Point, parseWaypointSub)
# Create a publisher to relay the waypoints received to the controller
waypointPub = rospy.Publisher("/pos_controller/orders", Order, queue_size=5)

# Create a publisher to relay the status of the UGV to the server
statusPub = rospy.Publisher("/ugv_check", Int16, queue_size=5)
# Create a subscriber to receive status updates from the position controller
statusSub = rospy.Subscriber("/pos_controller/status", Int16, parseStatusSub)

# Set looping rate
rate = rospy.Rate(20)

while not rospy.is_shutdown():
    sendWaypoint(waypointPub)
    sendStatus(statusPub)
    rate.sleep()
