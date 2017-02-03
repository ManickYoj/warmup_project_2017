#!/usr/bin/env python

import rospy, math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump

class PersonFollower(object):
    def __init__(self, debug = True):
        rospy.init_node('person_follower')
        rospy.Subscriber('/stable_scan', LaserScan, self.processScan)
        rospy.Subscriber('/bump', Bump, self.emergencyStop)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(40)
        self.debug = debug
        while not rospy.is_shutdown():
            r.sleep()

    def processScan(self, scan):
        frontScan = scan.ranges[-45:-1] + scan.ranges[0:46]
        print len(frontScan)
        return
        blobs = []
        r = []
        for i in range(0,90):
            if frontScan[i] > 0.0 and frontScan[i] < 3.0:       
                angle = i
                if i < 45:
                    angle = 360 - i 
                angle = math.radians(angle)
                x = math.cos(angle)*frontScan[i]
                y = math.sin(angle)*frontScan[i]
                r.append((x,y))
            else:
                blobs.append(r)
                r = []
        if self.debug:
            print blobs


    def emergencyStop(self, b):
        bumpList = [b.leftFront, b.leftSide, b.rightFront, b.rightSide]
        for reading in bumpList:
            if reading:
                self.stop = True


if __name__ == "__main__":
    test = PersonFollower()
