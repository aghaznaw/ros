#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
  x = msg.pose.pose.position.x
  y = msg.pose.pose.position.y
  print("x = "+str(x)+" y="+str(y))
  text_file = open("Location.txt", "w")
  text_file.write(str(x)+", "+str(y))
  text_file.close()

rospy.init_node('get_loc')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
