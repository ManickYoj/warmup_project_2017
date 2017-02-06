#!/usr/bin/env python
import utils
import rospy, math
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3, Point
from neato_node.msg import Bump

class PersonFollower(object):
    def __init__(self, debug = True):
        rospy.init_node('person_follower')
        # rospy.Subscriber('/stable_scan', LaserScan, self.processScan)
        rospy.Subscriber('/projected_stable_scan', PointCloud, self.processScan)
        rospy.Subscriber('/odom', Odometry, self.processOdom)
        rospy.Subscriber('/bump', Bump, self.emergencyStop)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisherCentroid = rospy.Publisher('/centroid', PointCloud, queue_size=10)
        self.pubCentroid = None
        r = rospy.Rate(40)
        self.debug = debug
        self.robotPos = (0,0,0)
        #inital instruction
        self.instruction = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
          )
        self.stop = False
        while not rospy.is_shutdown():
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
        pointsInRange = []
        # for i in range(len(pointsList)):
        #     if indicesList[i] < 46 or indicesList[i] >= 315:
        #         pointsInRange.append(pointsList[i])
        # for point in pointsInRange:
        #     if self.calcDistance((point.x, point.y), self.robotPos) > 1.5:
        #         pointsInRange.remove(point)
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
                print diffAng
                centroidList.append(curCentroid)
        pc = PointCloud()
        pc.header = scan.header
        pc.points = [Point(p[0], p[1], 0) for p in centroidList]
        self.pubCentroid = pc


        # for i in range(len(pointsInRange)):
        #     if i == 0:
        #         blob.append(pointsInRange[i])
        #     else:
        #         if self.calcPointDistance(pointsInRange[i], pointsInRange[i-1]) < 0.1:
        #             blob.append(pointsInRange[i])
        #         else:
        #             blobs.append(blob)
        #             blob = [pointsInRange[i]]
        #     if i == len(pointsInRange) - 1 and len(blob) != 0:
        #         blobs.append(blob)
        
        # frontScan = scan.ranges[-45:-1] + scan.ranges[0:46]
        # blobs = []
        # r = []
        # for i in range(0,90):
        #     if frontScan[i] > 0.0 and frontScan[i] < 1.5:       
        #         angle = i
        #         if i < 45:
        #             angle = 360 - i 
        #         angle = math.radians(angle)
        #         x = math.cos(angle)*frontScan[i]
        #         y = math.sin(angle)*frontScan[i]
        #         r.append((x,y))
        #     elif r != []:
        #         if (len(r) > 2 and len(r) < 15):
        #             # blobs.append(r)
        #             for p in r:
        #                 blobs.append(p)
        #         r = []
        # self.calCentroid(blobs)

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

    def calcPointDistance(self, p1, p2):
        return math.sqrt((p1.x-p2.x)**2 + ((p1.y-p2.y)**2))

    def processOdom(self, odom):
        robotOdom = odom.pose.pose
        self.robotPos = utils.poseToXYTheta(robotOdom)

if __name__ == "__main__":
    test = PersonFollower()
