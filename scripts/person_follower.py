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
        #inital instruction
        self.instruction = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
          )
        self.stop = False
        while not rospy.is_shutdown():
            if self.stop:
                self.publisher.publish(
                  Twist(
                    linear=Vector3(0, 0, 0),
                    angular=Vector3(0, 0, 0)
                  )
                )
            else:
                self.publisher.publish(self.instruction)
            r.sleep()

    def processScan(self, scan):
        frontScan = scan.ranges[-45:-1] + scan.ranges[0:46]
        blobs = []
        r = []
        for i in range(0,90):
            if frontScan[i] > 0.0 and frontScan[i] < 1.5:       
                angle = i
                if i < 45:
                    angle = 360 - i 
                angle = math.radians(angle)
                x = math.cos(angle)*frontScan[i]
                y = math.sin(angle)*frontScan[i]
                r.append((x,y))
            elif r != []:
                if (len(r) > 2 and len(r) < 15):
                    # blobs.append(r)
                    for p in r:
                        blobs.append(p)
                r = []
        self.calCentroid(blobs)

    def emergencyStop(self, b):
        bumpList = [b.leftFront, b.leftSide, b.rightFront, b.rightSide]
        for reading in bumpList:
            if reading:
                self.stop = True

    def calCentroid(self, blobs):
        x = 0
        y = 0
        for p in blobs:
            x += p[0]
            y += p[1]
        if len(blobs) == 0:
            return
        self.centroid = (x/len(blobs), y/len(blobs))
        if self.prevCentroids == None:
            self.prevCentroids = self.centroid
            return
        self.calcHeading()

        if self.debug:
            # for blob in blobs:
            #     print len(blob)
            print "prevCentroid: ", self.prevCentroids
            print "centroid: ", self.centroid
            print '-------------------------------------------------------------------'

    def calcHeading(self):
        if(self.calcDistance(self.prevCentroids, self.centroid) > 0.2):
            x = self.centroid[0]
            y = self.centroid[1]
            norm = math.sqrt(x**2 + y**2)
            if norm==0:
                self.instruction = (Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)))
                return
            xNorm = x/norm
            yNorm = y/norm
            
            # Difference b/w desired and actual heading
            diffAngle = math.atan2(yNorm, xNorm)

            xVel = self.calcDistance((0,0), (x,y))/2
            if xVel < 0.3:
                xVel = 0

            if self.debug:
                print "diffAngle: ", diffAngle
                print "xVel: ", xVel
                print "-----------------"
            self.instruction = Twist(linear=Vector3(xVel, 0, 0), angular=Vector3(0, 0, diffAngle))


    def calcDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + ((p1[1]-p2[1])**2))


if __name__ == "__main__":
    test = PersonFollower()
