#!/usr/bin/env python
import rospy
import std_msgs.msg 
import visualization_msgs.msg as vm
import geometry_msgs.msg as gm
import time

def clear():
    pub = rospy.Publisher('visualization_marker',vm.Marker,latch=True,queue_size=10)
    if not rospy.is_shutdown():
        m=vm.Marker()
        m.type = m.SPHERE_LIST
        m.action = m.DELETE
        pub.publish(m)

def talker():
    pub = rospy.Publisher('visualization_marker',vm.Marker,queue_size=10)
    rospy.init_node('waypoint_course')    
    frame = rospy.get_param('~frame','odom')
    dist = rospy.get_param('~size',2)
    XX = [0,dist,-dist,-dist,dist]
    YY = [0,dist,dist,-dist,-dist]
    m=vm.Marker()
    m.type = m.SPHERE_LIST
    m.action = m.ADD
    s=0.2
    m.scale.x = s
    m.scale.y = s
    m.scale.z = s
    m.color.r=1.0
    m.color.g=0
    m.color.a=1.0
    m.pose.orientation.w=1.0;
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = frame
    m.id = 1
    for x,y,mid in zip(XX,YY,range(len(XX))):
        p = gm.Point()
        p.x =x
        p.y =y
        m.points.append(p)
    cnt = 0
    while ( (cnt<25) and (not rospy.is_shutdown()) ):
        print("Publishing Sphere List")
        pub.publish(m)
        rospy.sleep(0.5)
        cnt += 1
      
      

if __name__ == '__main__':

    #time.sleep(10.0) # wait for rviz to start!
    try:
        #clear()
        talker()
    except rospy.ROSInterruptException:
        pass
