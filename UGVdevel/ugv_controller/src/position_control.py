#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Twist, Vector3, Point
from nav_msgs.msg import Odometry
from ugv_controller.msg import Order
from std_msgs.msg import Int16

x = 0.0
y = 0.0
theta = 0.0
idle = True
targetX = 0.0
targetY = 0.0



orderReceived = False
reachedTarget = False
orderList = Order()
targetX = 0.0
targetY = 0.0

# Variable to send UGV status
ugvStatus = 0

class StateMachine():
    def __init__(self):
        self.state = "waiting"
        self.pub = rospy.Publisher("/sim_p3at/cmd_vel", Twist, queue_size=1)

        self.kp = 0.1
        self.ki = 0.005
        self.kd = 0.005

        self.previous_error = 0
        self.previous_time = 0

        self.cp = 0
        self.ci = 0
        self.cd = 0

        self.started = False

    def transition(self, orderList):
        global ugvStatus
        global orderReceived
        global reachedTarget
        if self.state == "waiting":
            if orderReceived:
                orderReceived = False
                try:
                    newOrder = orderList
                    if newOrder.order == "move":
                        ugvStatus = 1
                        self.state = "moving"
                        self.targetX = newOrder.point.x
                        self.targetY = newOrder.point.y
                        print("Received order to move")
                except:
                    print("Error 1 reading message")
        elif self.state == "moving":
            if reachedTarget:
                self.state = "waiting"
                ugvStatus = 2
                print("Reached target")
            elif orderReceived:
                orderReceived = False
                try:
                    newOrder = orderList.pop(0)
                    if newOrder.order == "stop":
                        self.state = "waiting"
                        print("Shutdown order")
                except:
                    print("Error 2 reading message")


    def action(self, orderList, x, y, theta):
        global reachedTarget
        if self.state == "waiting":
            print("Waiting...")
            speed = Twist()
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            self.pub.publish(speed)
        if self.state == "moving":
            print("Moving...")
            inc_x = self.targetX - x
            inc_y = self.targetY - y
            angle_to_goal = math.atan2(inc_y, inc_x)
            error = math.sqrt(inc_x*inc_x+inc_y*inc_y)
            print("Error is", error)
            speed = Twist()
            if abs(angle_to_goal-theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
                self.pub.publish(speed)
            elif error > 0.31:
                speed = self.pidControl(error)
                speed.angular.z = 0.0
                #speed.linear.x = 0.3
                #speed.angular.z = 0.0
                self.pub.publish(speed)
            else:
                print("Reached")
                self.started = False
                self.ci = 0
                reachedTarget = True

    def pidControl(self, error):

        if not self.started:
            self.previous_time = rospy.Time.now()
            self.started = True

        speed = Twist()
        current_time = rospy.Time.now()
        dt = current_time-self.previous_time
        dt = dt.secs
        de = error - self.previous_error

        self.cp = error
        self.ci += error*dt
        if dt is not 0:
            self.cd = de/dt
        else:
            self.cd = 0
        speed.linear.x = self.kp*self.cp + self.ki*self.ci + self.kd*self.cd

        self.previous_error = error
        self.previous_time = current_time
        return speed


def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = tf.transformations.euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def parseOrder(msg):
    global orderReceived, orderList
    print("Receiving order")
    orderList = msg
    orderReceived = True

def sendStatus(publisher):
    """Send status to ugv_server"""
    global ugvStatus
    status = Int16()
    status.data = ugvStatus
    publisher.publish(status)

rospy.init_node("pos_controller")

orderSub = rospy.Subscriber("/sim_p3at/odom", Odometry, newOdom)
posSub = rospy.Subscriber("/pos_controller/orders", Order, parseOrder)

# Publisher to send ugvStatus to server
statusPub = rospy.publisher("/pos_controller/status", Int16, queue_size=5)

r = rospy.Rate(20)

controlMachine = StateMachine()

while not rospy.is_shutdown():
    controlMachine.transition(orderList)
    controlMachine.action(orderList, x, y, theta)
    sendStatus(ugvStatus)
    r.sleep()
