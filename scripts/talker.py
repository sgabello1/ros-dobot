#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import time
from math import sin, cos

def talker():
    pub = rospy.Publisher('geometry_pose', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(0.5) # 10hz

    t = 0
    a = 1
    while t < 25:
        msg = Pose()

        r = a*t
        msg.position.x = a*r*cos(t) + 180
        msg.position.y = a*r*sin(t) + 150
        msg.position.z = -65
        pub.publish(msg)
        time.sleep(0.1)
        t += 0.1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
