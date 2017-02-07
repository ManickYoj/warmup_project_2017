#!/usr/bin/env python

import rospy, math
import automaton as a

class Pursuer(a.Behavior):
	def __init__(self, debug=False):
		super(Pursuer, self).__init__("pursuer", debug)

		self.switchState = False
    self.prevCentroids = None

	def onBump(self, b):
		anyBump = False
		for reading in [
			b.leftFront,
			b.leftSide,
			b.rightFront,
			b.rightSide
		]:
			if reading:
				anyBump = True

		if anyBump:
			if not switchState:
				self.setSpeed(0, 0)
				self.switchState = True
			else:
				self.changeState("pursued")

	def onProjectedScan(self, scan):
		pass
    # centroids = [calCentroid(c) for c in blobs]

  def calcCentroid(self, blob):
  	cent_x, cent_y = (0, 0)

  	for point in blob:
  		x, y, _ = point
  		cent_x += x
  		cent_y += y

  	norm = len(blob)
  	return (cent_x / norm, cent_y / norm)

  def calcHeading(self):
  	pass

class Persued(a.Behavior):
	def __init__(self, debug=False):
		super(Pursued, self).__init__("pursued", debug)

		# Init behavior variables
		self.switchState = False
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
				anyBump = True

		if anyBump:
			if not switchState:
				self.setSpeed(0, 0)
				self.switchState = True
			else:
				self.changeState("pursuer")

	def onStableScan(self, scan):
		if self.switchState = True:
			return

		x, y = (25, 0)

		# Sum repulsive forces from objects into a
		# desired direction vector (away from objects)
		for angle, dist in enumerate(scan.ranges):
			if dist==0:
				continue

			mag = -1/(dist-0.2)**4
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

		# Calculate a linear speed proportional to how
		# close the robot's direction is to the desired direction
		xVel = (
			self.cutoffAngle-math.fabs(diffAngle)
		) / (self.cutoffAngle)

		# But avoid reversing the robot
		if xVel < 0:
			xVel = 0

		self.setSpeed(xVel, diffAngle*self.gain)


if __name__ == "__main__":
	# Construct an automaton with the pursuer and
	# pursued behaviors
	a.Automaton(
		name = "robot_tag",
		states = {
			"pursued": Pursued(),
			"pursuer": Pursuer()
		},
		initialState="pursued",
		markerTopics=[
			"/plan/direction",
			"/direction"
		]
	)