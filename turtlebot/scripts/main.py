#test
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionLib
from actionLib_msgs.msg import *
import math
import logging
import time
def getLocation():
	currentLocation = "0.0.0"
	gazebo::math::Pose
	return currentLocation

def main():
	try:
		pathFind()
	except: rospy.ROSInterruptException:
	rospy.initialize
	print "I am working!"
	#initialize prog
	roboLocation = getLocation()
	print roboLocation
	#Ask where to go on map

	targetCoordinates = input("Give target coordinates by using the following example: \n x,y,z #100,50,0")
	print targetCoordinates
main()
