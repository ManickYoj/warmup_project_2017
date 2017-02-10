#!/usr/bin/env python

import rospy, math, utils
import automaton as a
from geometry_msgs.msg import Point


class Pursuer(a.Behavior):
	def __init__(self, debug=False):
		super(Pursuer, self).__init__("pursuer", debug)

		self.centroids = []
		self.blobIncludeDist = 0.1

	def onBump(self, b):
		anyBump = False
		for reading in [
			b.leftFront,
			b.leftSide,
			b.rightFront,
			b.rightSide
		]:
			if reading:
				self.changeState("reverse", {"nextBehavior": "pursued"})

	def onProjectedScan(self, scan):
		points = scan.points
		if not points:
			return

		blobs = [[points[0]]]
		prevPoint = points[0]

		# Group points into blobs
		for point in points:
			if utils.dist(point, prevPoint) < self.blobIncludeDist:
				blobs[-1].append(point)
			else:
				blobs.append([point])

		# Join the first and last blob if their points are contiguous
		if utils.dist(blobs[0][0], blobs[-1][-1]) < self.blobIncludeDist:
			blobs[0].extend(blobs[-1])
			blobs = blobs[:-1]

		self.prevCentroids = self.centroids
		self.centroids = [self.calcCentroid(c) for c in blobs]

	def onOdom(self, odom):
		pose = odom.pose.pose

		# Create a list of candidate points from the centroids
		# that appear in the robot's front arc
		candidates = filter(
			lambda cent: self.isInFrontArc(cent, pose),
			self.centroids
		)

		# TODO: Figure out how to make this work
		# self.updateMarker("/think/candidates",
		# 	utils.marker(
		# 		markerType="POINTS",
		# 	)
		# )

		# Choose the nearest candidate point as the target
		minDist = float('Inf')
		target = None
		for cand in candidates:
			d = utils.dist(pose.position, cand)
			if d < minDist:
				minDist = d
				target = cand

		if target:
			self.updateMarker("/plan/target",
				utils.marker(
					markerType="SPHERE",
					position=(target.x, target.y, target.z),
					frame="/odom",
				)
			)

			angle = self.angleFromRobot(target, pose)
			self.setSpeed(0.5, -angle)
		else:
			self.setSpeed(0, 0)


	def isInFrontArc(self, point, pose, maxAngle=45, maxDist=1):
		maxAngle = math.radians(maxAngle)
		distance = utils.dist(pose.position, point)
		angle = self.angleFromRobot(point, pose)

		return angle < maxAngle and distance < maxDist

	def angleFromRobot(self, point, pose):
		x, y, robotTheta = utils.poseToXYTheta(pose)
		pointTheta = math.atan2(point.y-y, point.x-x)
		return utils.diffAngle(robotTheta, pointTheta)

	def calcCentroid(self, blob):
		cent = Point()

		for point in blob:
			cent.x += point.x
			cent.y += point.y

		norm = len(blob)
		return Point(cent.x / norm, cent.y / norm, 0)


class Pursued(a.Behavior):
	def __init__(self, debug=False):
		super(Pursued, self).__init__("pursued", debug)

		# Init behavior variables
		self.gain = rospy.get_param("~gain", 0.5)
		self.cutoffAngle = math.radians(
			rospy.get_param("~cutoff_angle", 180)
		)

	def onBump(self, b):
		anyBump = False
		for reading in [
			b.leftFront,
			b.leftSide,
			b.rightFront,
			b.rightSide
		]:
			if reading:
				self.changeState("reverse", {"nextBehavior": "pursuer"})

	def onStableScan(self, scan):
		x, y = (25, 0)

		# Sum repulsive forces from objects into a
		# desired direction vector (away from objects)
		for angle, distance in enumerate(scan.ranges):
			if distance==0:
				continue

			mag = -1/(distance-0.2)**4
			angle = math.radians(angle)

			x += math.cos(angle)*mag
			y += math.sin(angle)*mag

		# Normalize the direction vector to magnitude 1
		norm = math.sqrt(x**2 + y**2)
		if norm == 0:
			self.setSpeed(1, 0)
			return

		x /= norm
		y /= norm

		# Calculate the difference between the current and
		# desired direction
		diffAngle = math.atan2(y, x)

		# Visualize desired direction of robot
		self.updateMarker("/plan/direction",
			utils.marker(
				theta=diffAngle,
				markerType="ARROW",
				rgba=(0, 0, 1.0, 1.0),
				scale=(.9, 0.1, 0.1)
			)
		)

		# Visualize actual direction of robot
		self.updateMarker("/direction",
			utils.marker(
				theta=0,
				markerType="ARROW",
				rgba=(1.0, 0, 0, 1.0),
				scale=(0.9, 0.1, 0.1)
			)
		)

		# Calculate a linear speed proportional to how
		# close the robot's direction is to the desired direction
		xVel = (
			self.cutoffAngle-math.fabs(diffAngle)
		) / (self.cutoffAngle)

		# But avoid reversing the robot
		if xVel < 0:
			xVel = 0

		self.setSpeed(xVel, diffAngle*self.gain)

class Reverse(a.Behavior):
	def __init__(self, debug=False):
		super(Reverse, self).__init__("reverse", debug)
		self.nextBehavior = "pursued"
		self.startPoint = None

	def resume(self, args):
		if "nextBehavior" in args:
			self.nextBehavior = args["nextBehavior"]

		self.startPoint = None
		self.setSpeed(-0.5, 0)


	def onOdom(self, odom):
		pos = odom.pose.pose.position

		if self.startPoint is None:
			self.startPoint = pos
		elif utils.dist(self.startPoint, pos) > 0.25:
			self.changeState(self.nextBehavior)



if __name__ == "__main__":
	# Construct an automaton with the pursuer and
	# pursued behaviors
	a.Automaton(
		name = "robot_tag",
		states = {
			"pursued": Pursued(debug=True),
			"pursuer": Pursuer(debug=True),
			"reverse": Reverse(debug=True),
		},
		initialState="pursued",
		markerTopics=[
			"/plan/direction",
			"/direction",
			"/think/candidates",
			"/plan/target",
		],
		debug = True
	)