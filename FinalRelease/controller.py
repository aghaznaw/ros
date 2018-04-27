#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt, pi, pow, acos
from sensor_msgs.msg import LaserScan
import tf
import numpy as np

class PathFollower():

	def __init__(self):
		self.LIDAR_ERR = 0.05
		self.tf_listener = tf.TransformListener()
		self.odom_frame = '/odom'
		#rospy.on_shutdown(self.shutdown)
		self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		position = Point()
		move_cmd = Twist()
		try:
			self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
			self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			try:
				self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("There is an error!")

	def get_odom(self):
		try:
			(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
			rotation = euler_from_quaternion(rot)

		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")
			return

		return (Point(*trans), rotation[2])

	def moveRobot(self, path):
		 path = path
		 pathlenght = 0
		 r = rospy.Rate(10)
		 (position, rotation) = self.get_odom()

		 self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
		 position = Point()
		 move_cmd = Twist()
		 (position, rotation) = self.get_odom()
		 last_rotation = 0
		 linear_speed = 1
		 angular_speed = 1
		 pathlenght = 0
		 while(pathlenght < len(path)):
			 goal_x = path[pathlenght][0]
			 goal_y = path[pathlenght][1]
			 goal_z = 0
			 if goal_z > 180 or goal_z < -180:
				 print("you input worng z range.")
				 self.shutdown()
			 goal_z = np.deg2rad(goal_z)
			 goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
			 distance = goal_distance
			 while distance > 0.05:
				 (position, rotation) = self.get_odom()
				 x_start = position.x
				 y_start = position.y
				 path_angle = atan2(goal_y - y_start, goal_x- x_start)
				 print("goal (" +str(goal_x) +", "+str(goal_y) +")" )
				 if path_angle < -pi/4 or path_angle > pi/4:
					 if goal_y < 0 and y_start < goal_y:
						 path_angle = -2*pi + path_angle
					 elif goal_y >= 0 and y_start > goal_y:
						 path_angle = 2*pi + path_angle
				 if last_rotation > pi-0.1 and rotation <= 0:
					 rotation = 2*pi + rotation
				 elif last_rotation < -pi+0.1 and rotation > 0:
					 rotation = -2*pi + rotation
				 move_cmd.angular.z = angular_speed * path_angle-rotation

				 distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
				 move_cmd.linear.x = min(linear_speed * distance, 0.1)

				 if move_cmd.angular.z > 0:
					 move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
				 else:
					 move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

				 last_rotation = rotation
				 self.cmd_vel.publish(move_cmd)
				 r.sleep()

			 (position, rotation) = self.get_odom()

			 while abs(rotation - goal_z) > 0.05:
				 (position, rotation) = self.get_odom()
				 if goal_z >= 0:
					 if rotation <= goal_z and rotation >= goal_z - pi:
						 move_cmd.linear.x = 0.00
						 move_cmd.angular.z = 0.5
					 else:
						 move_cmd.linear.x = 0.00
						 move_cmd.angular.z = -0.5
				 else:
					 if rotation <= goal_z + pi and rotation > goal_z:
						 move_cmd.linear.x = 0.00
						 move_cmd.angular.z = -0.5
					 else:
						 move_cmd.linear.x = 0.00
						 move_cmd.angular.z = 0.5
				 self.cmd_vel.publish(move_cmd)
				 r.sleep()
			 rospy.loginfo("Stopping the robot...")
			 self.cmd_vel.publish(Twist())
			 pathlenght = pathlenght + 1

	def shutdown(self):
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

	def avoidObstacles(self):

		sub = rospy.Subscriber('/odom', Odometry, self.newOdom)
		pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

		speed = Twist()
		speed.angular.z = 0.0
		speed.linear.x = 0.0

		goal = Point()

		r = rospy.Rate(10)
		while not rospy.is_shutdown():

			msg = rospy.wait_for_message("/scan", LaserScan)
			self.scan_filter = []
			for i in range(360):
				if i <= 15 or i > 335:
					if msg.ranges[i] >= self.LIDAR_ERR:
						self.scan_filter.append(msg.ranges[i])

			if min(self.scan_filter) < 0.4:
				print(len(msg.ranges))
				print("Value at 0 degree" + str(msg.ranges[0]))
				print("Value at 90 degree" + str(msg.ranges[90]))
				print("Value at -90 degree" + str(msg.ranges[270]))
				return False
			else:
				rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))
				return True

	def joinNighberPoints(self, path):
		newpath = []
		for i in range(len(path)-1):
			x0 = path[i][0]
			x1 = path[i+1][0]
			y0 = path[i][1]
			y1 = path[i+1][1]

			dist = sqrt(pow((x1 - x0), 2) + pow((y1 - y0), 2))

			if(dist > 0.2):
				newpath.append([i])
				newpath[i].append([x0])
				newpath.append([i])
				newpath[i].append([y0])
			#print("dist of " + str(i) + " and " + str(i+1) +" is "+ str(dist))
		#print(newpath)




	def setPath(self):

		lineNum = sum(1 for line in open('PathArray.txt'))

		w, h = 2, lineNum
		i = 0
		goalArray = [[0 for x in range(w)] for y in range(h)]
		with open("PathArray.txt") as f:
			for line in f:
				lx =  line.split(",")
				goalArray[i][0] = float(lx[0])
				goalArray[i][1] = float(lx[1])
				i += 1


		#self.joinNighberPoints(goalArray)
		self.moveRobot(goalArray)


def main():
    rospy.init_node('controller')
    try:
		pathFollower = PathFollower().setPath()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
