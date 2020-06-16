#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist

from threading import Timer
import time

class Wdog(object):
    def __init__(self,tout):
        self.tout = tout
        topic = "my_p3at/cmd_vel"
        rospy.Subscriber(topic, Twist, self.callback)
        self.pub = rospy.Publisher(topic, Twist, queue_size=10)
        self.timer = Timer(self.tout,self.watch)
        self.timer.daemon=True
        self.timer.start()
        self.N = 0

    def watch(self):
        if self.N < 1:
            rospy.loginfo("Timeout!")
            msg = Twist()
            self.pub.publish(msg)
        time.sleep(0.01)
        self.restart()

    def restart(self):
        self.N = 0
        self.timer = Timer(self.tout,self.watch)
        self.timer.daemon=True
        self.timer.start()

    def callback(self,data):
        self.N += 1
        

if __name__ == '__main__':

    rospy.init_node('twist_watchdog', anonymous=True)
    wdog = Wdog(1.0)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

