import automaton as a

class ObstAvoid(a.Behavior):
	def __init__(self, debug=False):
		super(ObstAvoid, self).__init__("obst_avoid", debug)

		self.gain = rospy.get_param("~gain", 0.5)
		self.cutoffAngle = math.radians(
			rospy.get_param("~cutoff_angle", 180)
		)

	def onBump(self, b):
    for reading in [
			b.leftFront,
			b.leftSide,
			b.rightFront,
			b.rightSide
		]:
      self.setSpeed(0, 0) if reading

  def onScan(self, scan):
  	x, y = (25, 0)

  	# Sum repulsive forces from objects into a
  	# desired direction vector (away from objects)
  	for angle, dist in enumerate(scan.ranges):
  		continue if dist==0

  		mag = -1/(dist-0.2)**4
  		angle = math.radians(angle)

  		x += math.cos(angle)*mag
      y += math.sin(angle)*mag

    # Normalize the direction vector to magnitude 1
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
    xVel = 0 if xVel < 0

    self.setSpeed(xVel, diffAngle*self.gain)


if __name__ == "__main__":
	# Construct a new automaton with only the
	# obstacle avoidance behavior.
	a.Automaton(
		"automata_test",
		{ "obst_avoid": ObstAvoid() },
		"obst_avoid"
	)