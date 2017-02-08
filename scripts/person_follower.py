#!/usr/bin/env python
import utils
import rospy, math
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Point, PointStamped
from neato_node.msg import Bump

class PersonFollower(object):
    def __init__(self, debug = True):
        rospy.init_node('person_follower')
        # rospy.Subscriber('/stable_scan', LaserScan, self.processScan)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.processScan)
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        rospy.Subscriber('/bump', Bump, self.emergencyStop)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisherTar = rospy.Publisher('/target', PointStamped, queue_size=10)
        self.publisherCentroid = rospy.Publisher('/centroid', PointCloud, queue_size=10)
        self.pubCentroid = None
        r = rospy.Rate(40)
        self.debug = debug
        self.robotPos = (0,0,0)
        self.target = None
        self.prevCentroids = []
        self.prevPrevCentroids = []
        #inital instruction
        self.instruction = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
          )
        self.stop = False
        while not rospy.is_shutdown():
            if self.target != None:
                self.publisherTar.publish(PointStamped(self.header, Point(self.target[0],self.target[1], 0)))
            if self.pubCentroid != None:
                self.publisherCentroid.publish(self.pubCentroid)
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
        pointsList = scan.points
        indicesList = scan.channels[1].values
        blobs = []
        blob = []
        for i in range(len(pointsList)):
            if i == 0:
                blob.append(pointsList[i])
            else:
                if self.calcPointDistance(pointsList[i], pointsList[i-1]) < 0.1:
                    blob.append(pointsList[i])
                else:
                    blobs.append(blob)
                    blob = [pointsList[i]]
            if i == len(pointsList) - 1 and len(blob) != 0:
                    blobs.append(blob)
        centroidList = [] 
        for b in blobs:
            curCentroid = self.calCentroid(b)
            ang1 = math.atan2((curCentroid[1] - self.robotPos[1]),(curCentroid[0] - self.robotPos[0]))
            diffAng = utils.diffAngle(ang1, self.robotPos[2])
            if math.fabs(diffAng) < math.pi/4 and self.calcDistance(self.robotPos, curCentroid) < 1:
                centroidList.append(curCentroid)
        
        pc = PointCloud()
        pc.header = scan.header
        self.header = pc.header
        pc.points = [Point(p[0], p[1], 0) for p in centroidList]
        self.pubCentroid = pc
        self.calcMoving(centroidList)

    def emergencyStop(self, b):
        bumpList = [b.leftFront, b.leftSide, b.rightFront, b.rightSide]
        for reading in bumpList:
            if reading:
                self.stop = True

    def calCentroid(self, blob):
        x = 0
        y = 0
        for p in blob:
            x += p.x
            y += p.y
        if len(blob) == 0:
            return
        return (x/len(blob), y/len(blob))
 
     
    def calcMoving(self, centroidList):
        # if len(centroidList) == 0 or len(self.prevPrevCentroids) == 0 or len(self.prevCentroids) == 0:
        if len(centroidList) == 0 :
            # self.prevPrevCentroids = self.prevCentroids
            # self.prevCentroids = centroidList
            self.target = None
            self.calcHeading(self.target)
        else: 
            candidate = None
            isMoving = True
            # for c in centroidList:
            #     for p in self.prevCentroids:
            #         if self.calcDistance(c,p) < 0.05:
            #             isMoving = False
            #     for p in self.prevPrevCentroids:
            #         if self.calcDistance(c,p) < 0.05:
            #             isMoving = False
            #     if isMoving:
            #         if candidate == None:
            #             candidate = c
            #         else:
            #             if self.calcDistance(self.robotPos, candidate) > self.calcDistance(self.robotPos, c):
            #                 candidate = c
            for c in centroidList:
                if candidate == None or self.calcDistance(c, self.robotPos) < self.calcDistance(candidate, self.robotPos):
                    candidate = c
            self.target = candidate
            self.calcHeading(candidate)

    def calcHeading(self, tar):
        if tar == None:
            self.instruction = Twist(
                linear=Vector3(0, 0, 0),
                angular=Vector3(0, 0, 0)
              )
        else:
            # diffAngle = math.atan2(tar[1] - self.robotPos[1], tar[0] - self.robotPos[0])
            ang1 = math.atan2((tar[1] - self.robotPos[1]),(tar[0] - self.robotPos[0]))
            diffAngle = utils.diffAngle(ang1, self.robotPos[2])
            xVel = self.calcDistance(self.robotPos, (tar[0],tar[1]))
            targetDistance = 0.3
            # if xVel < 0.3:
            #     xVel = 0
            if self.debug:
                print "diffAngle: ", diffAngle
                print "xVel: ", xVel
                print "-----------------"
            self.instruction = Twist(linear=Vector3((xVel - targetDistance)*0.5, 0, 0), angular=Vector3(0, 0, diffAngle*0.5))

    def calcDistance(self, p1, p2):
        return math.sqrt((p1[0]-p2[0])**2 + ((p1[1]-p2[1])**2))

    def calcPointDistance(self, p1, p2):
        return math.sqrt((p1.x-p2.x)**2 + ((p1.y-p2.y)**2))

    def processOdom(self, odom):
        robotOdom = odom.pose.pose
        self.robotPos = utils.poseToXYTheta(robotOdom)

if __name__ == "__main__":
    test = PersonFollower()
