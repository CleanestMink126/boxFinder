#!/usr/bin/env python
import rospy
import math
# import st
import numpy as np
from std_msgs.msg import String
from irl.msg import *
import time
import getRedCircles
from irl.srv import arm_cmd

'''
run robot_write.launch
also run arm_node.py first
'''
pixelError = 10
tiltError = 0.05

class movementControl:
	moveMult = 1
	tiltMult = 5
	rectCorners = None
	stepSize = None
	sizeBoard = 4
	#-500 5400 -835
	def __init__(self,pos = None):
		if pos is None:
			self.pos = [-500,4000,5000]
		else:
			self.pos = pos

		self.wristPos = 3050


		# self.arm_pub = rospy.Publisher('arm_cmd', String, queue_size=10)

		# self.write_pub = rospy.Publisher('write_cmd', Edwin_Shape, queue_size=10)
	def hand(self):
		rospy.init_node('edwin_sudoku', anonymous = True)
		rospy.wait_for_service('arm_cmd', timeout=15)
		cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
		try:
			msg = "data: rotate_hand:: 3200"
			resp1 = cmd_fnc(msg)
			print "command done"
		except rospy.ServiceException, e:
		    print ("Service call failed: %s"%e)

	def move(self):
		rospy.wait_for_service('arm_cmd', timeout=15)
		cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
		print "I have requested the command"
		try:
			msg = "data: move_to:: " + str(self.pos[0]) + ", " + str(self.pos[1]) + ", " + str(self.pos[2]) + ", 0"
			resp1 = cmd_fnc(msg)
			print "command done"
		except rospy.ServiceException, e:
		    print ("Service call failed: %s"%e)

	def tilt(self):
		'''Values between 1 and 1'''

		rospy.wait_for_service('arm_cmd', timeout=15)
		cmd_fnc = rospy.ServiceProxy('arm_cmd', arm_cmd)
		print "I have requested the command"
		try:
			msg = "data: rotate_wrist:: " + str(self.wristPos)
			resp1 = cmd_fnc(msg)
			print "command done"
		except rospy.ServiceException, e:
		    print ("Service call failed: %s"%e)
		time.sleep(3)

	# def track

	def findPoint(self, circleObj, indexes):

		points = circleObj.permCorners[indexes]
		xDiff,yDiff = circleObj.getCenter(points)
		while abs(xDiff) > pixelError or abs(yDiff) > pixelError:
			self.pos[0] += int(xDiff * self.moveMult)
			self.pos[1] += int(yDiff * self.moveMult)
			self.move()
			# self.straightenRect(circleObj)
			circleObj.runOnce()
			points = circleObj.permCorners[indexes]
			xDiff,yDiff = circleObj.getCenter(points)

			if not min(circleObj.valid[indexes]):
				return None, None

		return self.pos[0],self.pos[1]

	def findCenter(self,circleObj):
		return self.findPoint(circleObj, [0,1,2,3])

	def findLeft(self,circleObj):
		return self.findPoint(circleObj, [0,2])

	def findTop(self,circleObj):
		return self.findPoint(circleObj, [0,1])

	def straightenRect(self, circleObj):
		#positive result form circleObj means turn right
		tiltVal = circleObj.getRotation(circleObj.permCorners)
		while abs(tiltVal) > tiltError:
			self.wristPos -= tiltVal * self.tiltMult
			self.tilt()
			circleObj.runOnce()
			tiltVal = circleObj.getRotation(circleObj.permCorners)

	def getDims(self, circleObj):
		# centerx, centery = self.findCenter(circleObj)
		# print 'CENTER'
		# print 'x:',centerx,'y:',centery
		# topx, topy = self.findTop(circleObj)
		# print 'TOP'
		# print 'x:',topx,'y:',topy
		# leftx, lefty = self.findLeft(circleObj)
		# print 'LEFT'
		# print 'x:',leftx,'y:',lefty

		centerx, centery = self.findPoint(circleObj,[0])
		print 'Top Left'
		print 'x:',centerx,'y:',centery
		centerx, centery = self.findPoint(circleObj,[1])
		print 'Top Right'
		print 'x:',centerx,'y:',centery


		xlength = 2 * (centerx - leftx)
		ylength = 2 * (centery - topy)
		self.rectCorners = [(leftx,topy),(leftx + xlength,topy),(leftx,topy+ylength),(leftx + xlength,topy + ylength)]
		self.stepSize = (xlength/self.sizeBoard,ylength/self.sizeBoard)

	def goToLoc(x,y):
		'''Indexing from 0'''
		self.pos[0] = self.rectCorners[0][0] + x * self.stepSize[0]
		self.pos[1] = self.rectCorners[0][1] + y * self.stepSize[1]
		self.move()



def z_calculation(input_y):
	scaler = -735 - int((input_y - 4000)/9.4)
	return scaler


def run():
	# rospy.init_node('arm_tester', anonymous=True)
	# time.sleep(2)
	print "starting"



	# while not rospy.is_shutdown():
	# msg = Edwin_Shape()
	moveObj = movementControl()

	moveObj.move()
	time.sleep(3)
	moveObj.tilt()
	moveObj.hand()
	circleObj = getRedCircles.circleFinder(videoNum = 1)
	while circleObj.permCorners is None:
		circleObj.runOnce()
	# moveObj.getDims(circleObj)



	# msg.shape = "123456"
	# msg.x = -500
	# msg.y = 5700
	# msg.z = -835
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "7890!?"
	# msg.x = -500
	# msg.y = 5400
	# msg.z = -835
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "abcdef"
	# msg.x = -500
	# msg.y = 5700
	# msg.z = -790
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "ghijkl"
	# msg.x = -500
	# msg.y = 5400
	# msg.z = -790
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "mnopqr"
	# msg.x = -500
	# msg.y = 5100
	# msg.z = -780
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "stuvwx"
	# msg.x = -500
	# msg.y = 4800
	# msg.z = -770
	# pub.publish(msg)
	# time.sleep(5)
	#
	# msg.shape = "yz"
	# msg.x = -500
	# msg.y = 4500
	# msg.z = -770
	# pub.publish(msg)
	# time.sleep(5)

if __name__ == '__main__':
	run()
