#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt


class PathPlanner():

	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0

	def newOdom(self, msg):

		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y

		rot_q = msg.pose.pose.orientation
		(roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

	def moveTo(self, path):

		sub = rospy.Subscriber('/odom', Odometry, self.newOdom)
		pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

		speed = Twist()
		path = path
		goal = Point()


		r = rospy.Rate(4)
		i = 0
		while not rospy.is_shutdown():
			goal.x = path[i][0]
			goal.y = path[i][1]
			inc_x = goal.x - self.x
			inc_y = goal.y - self.y

			angle_to_goal = atan2(inc_y, inc_x)
			if abs(angle_to_goal - self.theta) > 0.3:
				speed.linear.x = 0.0
				speed.angular.z = 0.3
			else:
				speed.linear.x = 0.5
				speed.angular.z = 0.0
				destance = sqrt(((goal.x - self.x) * (goal.x - self.x)) + ((goal.y - self.y) - (goal.y - self.y)))
				if destance < 0.1:
					i = i + 1
					if i == len(path):
						return 0

			pub.publish(speed)
			r.sleep()

	def setPath(self):
		path = [[0, 2], [2, 4], [-4, 6], [10, 10]]
		self.moveTo(path)

def main():
    rospy.init_node('controller')
    try:

        pathplanner = PathPlanner().setPath()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
