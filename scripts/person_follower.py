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
        self.prevCentroids = None
        r = rospy.Rate(40)
        self.debug = debug
        while not rospy.is_shutdown():
            r.sleep()

    def processScan(self, scan):
        frontScan = scan.ranges[-45:-1] + scan.ranges[0:46]
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
            elif r != []:
                blobs.append(r)
                r = []
        self.calCentroid(blobs)

    def emergencyStop(self, b):
        bumpList = [b.leftFront, b.leftSide, b.rightFront, b.rightSide]
        for reading in bumpList:
            if reading:
                self.stop = True

    def calCentroid(self, blobs):
        centroids = []
        for blob in blobs:
            x = 0
            y = 0
            for point in blob:
                x += point[0]
                y += point[1]
            centroids.append(((x/len(blob)), (y/len(blob))))    

        if self.prevCentroids == None:
            self.prevCentroids = centroids
            return

        if self.debug:
            print centroids
            print '-------------------------------------------------------------------'

if __name__ == "__main__":
    test = PersonFollower()
