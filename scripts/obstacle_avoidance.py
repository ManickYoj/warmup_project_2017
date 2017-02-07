#!/usr/bin/env python

import rospy, math, utils
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from visualization_msgs.msg import Marker


class ObstacleAvoider(object):
    def __init__(self, debug=False):
        rospy.init_node('obstacle_avoider')
        rospy.Subscriber('/stable_scan', LaserScan, self.processScan)
        rospy.Subscriber('/bump', Bump, self.emergencyStop)
        self.planPub = rospy.Publisher('/plan/direction', Marker, queue_size=10)
        self.dirPub = rospy.Publisher('/direction', Marker, queue_size=10)
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Inital instruction
        self.instruction = Twist(
            linear=Vector3(0, 0, 0),
            angular=Vector3(0, 0, 0)
          )
        self.stop = False
        self.debug = debug
        self.cutoffAngle = math.radians(rospy.get_param("~cutoff_angle", 180))
        self.gain = rospy.get_param("~gain", 0.5)

        # Visualization variables
        self.plan = None
        self.direction = None

        r = rospy.Rate(40)
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

                if self.direction is not None:
                    self.dirPub.publish(self.direction)

                if self.plan is not None:
                    self.planPub.publish(self.plan)

            r.sleep()

    def emergencyStop(self, b):
        bumpList = [b.leftFront, b.leftSide, b.rightFront, b.rightSide]
        for reading in bumpList:
            if reading:
                self.stop = True

    def processScan(self, scan):
        """
        Convert the data from the scan into useful information;
        specifically, find the distances to the closest detected
        objects and pass their range and angle from
        the robot on to the controller.
        """

        x = 25
        y = 0

        for angle, dist in enumerate(scan.ranges):
            if dist==0:
                continue

            mag = -1/(dist-0.2)**4
            angle = math.radians(angle)

            x += math.cos(angle)*mag
            y += math.sin(angle)*mag

        if self.debug:
            print "x: ", x
            print "y: ", y
        norm = math.sqrt(x**2 + y**2)
        if norm==0:
            self.instruction = (Twist(linear=Vector3(1, 0, 0), angular=Vector3(0, 0, 0)))
            return
        x /= norm
        y /= norm


        # Difference b/w desired and actual heading
        diffAngle = math.atan2(y, x)

        # Visualize desired direction of robot
        self.direction = utils.marker(rgba=(1.0, 0, 0, 1.0), scale=(.9, 0.1, 0.1))
        self.plan = utils.marker(theta=diffAngle, scale=(.9, 0.1, 0.1))

        xVel = (self.cutoffAngle-math.fabs(diffAngle))/(self.cutoffAngle)
        if xVel<0:
            xVel=0
        if self.debug:
            print "diffAngle: ", diffAngle
            print "xVel: ", xVel
            print "-----------------"

        self.instruction = Twist(linear=Vector3(xVel, 0, 0), angular=Vector3(0, 0, diffAngle*self.gain))


if __name__ == "__main__":
    ObstacleAvoider(True)