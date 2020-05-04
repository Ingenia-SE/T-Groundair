#!/usr/bin/env python  
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    print "hi"
    rospy.init_node('test')

    listener = tf.TransformListener()


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', 
                                                   '/leader_tf/odom',
                                                   rospy.Time(0))
            print "tf"

        except (tf.LookupException, 
                tf.ConnectivityException, 
                tf.ExtrapolationException):
            continue
        print "?"
        print trans
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        #turtle_vel.publish(cmd)
        

        rate.sleep()
