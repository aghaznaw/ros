#test
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionLib
from actionLib_msgs.msg import *
import math
import logging
import time
#def exportGazeboMap():
	#exporting and parsing for the map, use git/PySDF for example
	#given for Jere to do

def getLocation():
	currentLocation = "0.0.0"
	#gazebo::math::Pose
	#if no location is known, ask the user to set a getLocation
	userDefinedLoc = input("Please give the robot a location")
	return currentLocation
def getPath():

	#View the map as a plane the algorithm understands how to follow
	#import the map from gazebo https://github.com/markwsilliman/turtlebot/blob/master/goforward_and_avoid_obstacle.py
	#listOfCommands = [goal.target_pose.pose.position.x = 3.0, goal.target_pose.pose.position.x = 10.0,]
	issueCommand = goalForThisLoop
	return issueCommand
def pathFind(tCoords):
	curLoc = getLocation() #current location
	#Generate a chain of commands the robot will follow. This chain of commands
	#will be generated with the pathfinding algorithm
	#"In order for the robot to reach destination X, it has to traverse..."
	#"...to point 1(first corner),  2(second corner)  and 3(destination)"
	#highest priority
	#move(1st point)
	#move(2nd point)
#def pathFindJereSolution

def main():
	targetCoordinates = input("Give target coordinates by using the following example: \n x,y #100,50")
	print targetCoordinates
	#try:
	pathFind(targetCoordinates)
	#except: rospy.ROSInterruptException: #interruption is the moment it stops moving
	rospy.initialize
	print "I am working!"
	#initialize prog
	roboLocation = getLocation()
	print roboLocation
	#Ask where to go on map


main()
