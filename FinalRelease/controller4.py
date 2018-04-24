#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, pi, pow, acos
from sensor_msgs.msg import LaserScan

class PathPlanner():

	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.LIDAR_ERR = 0.05

	def newOdom(self, msg):

		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y

		rot_q = msg.pose.pose.orientation
		(roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])



	def moveTo(self, path):

		sub = rospy.Subscriber('/odom', Odometry, self.newOdom)
		pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

		speed = Twist()
		speed.angular.z = 0.0
		speed.linear.x = 0.0

		goal = Point()

		path = path
		reached_goal = False
	 	j= 0
		r = rospy.Rate(20)
		while not rospy.is_shutdown():

			msg = rospy.wait_for_message("/scan", LaserScan)
			self.scan_filter = []
			for i in range(360):
				if i <= 15 or i > 335:
					if msg.ranges[i] >= self.LIDAR_ERR:
						self.scan_filter.append(msg.ranges[i])

			if min(self.scan_filter) < 0.2:
				print(len(msg.ranges))
				print("Value at 0 degree" + str(msg.ranges[0]))
				print("Value at 90 degree" + str(msg.ranges[90]))
				print("Value at -90 degree" + str(msg.ranges[270]))
				speed.linear.x = 0.0
				speed.angular.z = 0.0
				pub.publish(speed)
				rospy.loginfo('Stop!')
			else:
				rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))

				goal.x = path[j][0]
				goal.y = path[j][1]
				inc_x = goal.x - self.x
				inc_y = goal.y - self.y

				result = False

				angle_to_goal = atan2(inc_y, inc_x)

				print ("goalx:" + str(goal.x))
				print ("goaly:" + str(goal.y))
				print("theta:" + str(self.theta))
				print("angle-theta:" +str(angle_to_goal - self.theta))
				print("angle_to_goal:" + str(angle_to_goal))
				speed.angular.z = 0
				speed.linear.x = 0
				if abs(angle_to_goal - self.theta) > 0.2 and abs(angle_to_goal - self.theta) < 5.85:
					speed.linear.x = 0.0
					speed.angular.z = 0.3
				else:
					speed.linear.x = 0.5
					speed.angular.z = 0.0
					destance = sqrt(((goal.x - self.x) * (goal.x - self.x)) + ((goal.y - self.y) * (goal.y - self.y)))
					#print("distantce:" + str(destance))
					if destance < 0.2:
						speed.linear.x = 0.0
						speed.angular.z = 0.0
						j = j + 1
						if j == len(path):
							reached_goal = True;
				pub.publish(speed)
				r.sleep()
				if(reached_goal):
					return reached_goal
	def findAngle(self, p0x, p0y, p1x, p1y, p2x, p2y):

		a = pow(p1x - p0x, 2) + pow(p1y - p0y, 2)
  		b = pow(p1x - p2x, 2) + pow(p1y - p2y, 2)
	  	c = pow(p2x - p0x, 2) + pow(p2y - p0y, 2)

		degree = acos((a + b - c) / sqrt(4 * a * b)) * 180 / pi
		return degree

	def setAngleToGoal(self, goalx, goaly):

		sub = rospy.Subscriber('/odom', Odometry, self.newOdom)
		pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

		speed = Twist()
		goal = Point()


		r = rospy.Rate(6)
		i = 0
		while not rospy.is_shutdown():
			goal.x = goalx
			goal.y = goaly
			inc_x = goal.x - self.x
			inc_y = goal.y - self.y

			angle_to_goal = atan2(inc_y, inc_x)
			print(angle_to_goal - self.theta )
			print ("I:" + str(i))
			print ("goalx:" + str(goal.x))
			print ("goaly:" + str(goal.y))
			if abs(angle_to_goal) > 5:
				speed.angular.z = -0.2
			if abs(angle_to_goal - self.theta) > 0.05:
				speed.angular.z = 0.2
			else:
				speed.angular.z = 0;
			if abs(angle_to_goal - self.theta) < 0.05:
				return True

			pub.publish(speed)
			r.sleep()


	def setPath(self):
		#path = [[-2.5, 2.8], [1.7, 2.8], [1.7, -1.3], [6, -1.3]]
		#path = [[1.7, -1.3], [1.7, 2.8], [-2.5, 2.8], [-2.5, -0.5]]
		path = [[0.0, 2.2], [-2.6, 2.2], [-2.7, 2.2], [-2.7, 3.1], [-3.0, 3.1], [-3.1, 3.1], [-3.1, -3.0], [-1.1, -2.0], [-1.0, -2.0]]

		self.moveTo(path)

def main():
    rospy.init_node('controller')
    try:
		pathplanner = PathPlanner().setPath()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
