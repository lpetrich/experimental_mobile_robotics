#!/usr/bin/env python

import rospy
import actionlib
import sys
import numpy as np
import math
import time
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from std_msgs.msg import String, Bool
from kobuki_msgs.msg import Sound, Led
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan,Joy
from smach import State,StateMachine
import smach_ros
import cv2
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

def global_localize():
	rospy.wait_for_service('global_localization')
	localize_service = rospy.ServiceProxy('global_localization', Empty)

	try:
		res = localize_service()
	except:
		print("Service did not process request")

def joy_callback(msg):
	global x_pressed
	global y_pressed
	global move_base

	x_pressed = msg.buttons[0]
	y_pressed = msg.buttons[2]

	if y_pressed == 1:
		print 'B Pressed'
		move_base.cancel_goal()
		rospy.signal_shutdown('Y Pressed')

	if x_pressed == 1:
		print 'X Pressed'

def scan_callback(msg):
	global g_range_ahead

	largeDistance = 100000

	arrayLen = 201

	triPoint = [i for i in range (0,arrayLen)]

	for i in range(0,arrayLen):
		triPoint[i] = msg.ranges[(len(msg.ranges)/2) - ((arrayLen-1)/2) + i]

	# print triPoint

	g_range_ahead = largeDistance
	for i in range (0, len(triPoint)):
		if (triPoint[i] < g_range_ahead and not math.isnan(triPoint[i])):
			g_range_ahead = triPoint[i]

	if g_range_ahead == largeDistance:
		g_range_ahead = 0


def dock_complete_callback(data):
	print 'reached localize return from docking'
	global move_base, return_pose, pause
	check = data.data
	if check:
		print 'localize: docking complete -- returning to previous pose'
		# docking completed
		goal = goal_pose(return_pose)
		move(goal)
		pause = False

def dock_start_turn_callback(data):
	global move_base, return_pose, current_pose, pause, tag_areas, tag_type, soundhandle
	check = data.data
	if check and not pause:
		# docking intialized
		print 'docking turn initialized'

		kickback = False
		for i in range(len(tag_areas)):
			a, b = tag_areas[i][0], tag_areas[i][1]
			d = math.sqrt((a-current_pose.pose.position.x)**2 + (b-current_pose.pose.position.y)**2)
			if d < 0.7:
				print 'this tag has already been found'
				print 'distance to found tag: ', d
				kickback = True
				already_found_pub.publish(True)


		print 'kickback: ',kickback
		if not kickback:
			already_found_pub.publish(False)
			if tag_type == 'UA':
				soundhandle.playWave('/home/hegberg/Music/Detroit Red Wings NHL.mp3')
			else:
				soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')
			return_pose = current_pose
			pause = True
			move_base.cancel_goal()
			rotate90()

def tag_type_callback(data):
	global tag_type
	tag_type = data.data

def move(goal):
	global move_base
	move_base.send_goal(goal)
	cap_time = move_base.wait_for_result(rospy.Duration(120))
	if not cap_time:
		move_base.cancel_goal()
	else:
		if is_close(goal):
			print 'close to goal'
			return #made it close enough, go to next goal
		else:
			rotateTime = rospy.Time.now() + rospy.Duration(3)

			while (rospy.Time.now() < rotateTime):
				# print rotateTime - rospy.Time.now()
				twist = Twist()
				twist.angular.z = 0.2
				twist.linear.x = -0.2
				cmd_vel_pub.publish(twist)
		# state = move_base.get_state()
		# if state == GoalStatus.SUCCEEDED:
		# 	print 'goal succeeded'
		# 	return
		# else:
		# 	print 'goal didnt succeed, wtf'
		# 	return

def is_close(goal):
	global current_pose
	a, b = goal.target_pose.pose.position.x, goal.target_pose.pose.position.y
	if current_pose != None:
		d = math.sqrt((a-current_pose.pose.position.x)**2 + (b-current_pose.pose.position.y)**2)
	else:
		d = 0.7
	return d < 0.2 #if close return true otherwise return false

def rotate90():
	global current_pose

	rotateTime = rospy.Time.now() + rospy.Duration(4.5)

	while (rospy.Time.now() < rotateTime):
		# print rotateTime - rospy.Time.now()
		twist = Twist()
		twist.angular.z = -0.4
		twist.linear.x = 0
		cmd_vel_pub.publish(twist)

	print 'finished turning'

	dock_ready.publish(True)

def goal_pose(return_pose):
	global tag_areas

	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = 'map'
	goal_pose.target_pose.header.stamp = rospy.Time.now()
	goal_pose.target_pose.pose.position.x = return_pose.pose.position.x
	goal_pose.target_pose.pose.position.y = return_pose.pose.position.y
	goal_pose.target_pose.pose.position.z = return_pose.pose.position.z
	goal_pose.target_pose.pose.orientation.w = return_pose.pose.orientation.w
	goal_pose.target_pose.pose.orientation.x = return_pose.pose.orientation.x
	goal_pose.target_pose.pose.orientation.y = return_pose.pose.orientation.y
	goal_pose.target_pose.pose.orientation.z = return_pose.pose.orientation.z

	tag_areas.append((return_pose.pose.position.x, return_pose.pose.position.y))
	print 'tag postions: ', tag_areas
	return goal_pose


class Start(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])
		# global state_change_time
		# cv2.namedWindow("window", 1)

	def execute(self, userdata):
		print 'Start'
		#print userdata.state_change_time_in

		global x_pressed, soundhandle
		# soundhandle.playWave('/home/hegberg/Music/Detroit Red Wings NHL.mp3')
		while x_pressed == 0:
			time.sleep(0.01)

		global_localize()

		#cv2.imshow("window", image)

		#time.sleep(1.01)

		return 'success'

class Localize(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

	def execute(self, userdata):

		global g_range_ahead

		rotateTime = rospy.Time.now() + rospy.Duration(10)
		forwardTime = rospy.Time.now() + rospy.Duration(12)

		count = 0

		while True:
			if g_range_ahead > 1.4 and (rospy.Time.now() < rotateTime):
				twist = Twist()
				twist.angular.z = 0.4
				twist.linear.x = 0
				cmd_vel_pub.publish(twist)
				# print 'circle'

			elif g_range_ahead < 1.51:
				twist = Twist()
				twist.angular.z = 0.3
				twist.linear.x = 0
				cmd_vel_pub.publish(twist)
				# print g_range_ahead

			elif g_range_ahead > 1.5 and (rospy.Time.now() < forwardTime):
				twist = Twist()
				twist.angular.z = 0
				twist.linear.x = 0.3
				cmd_vel_pub.publish(twist)
				# 'straight'

			elif count > 0:
				break

			else:
				rotateTime = rospy.Time.now() + rospy.Duration(10)
				forwardTime = rospy.Time.now() + rospy.Duration(12)
				count += 1

			# time.sleep(0.01)


		return 'success'

class TraverseMap(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

		global move_base, return_pose, current_pose

		self.atPoint = False
		move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		move_base.wait_for_server(rospy.Duration(60))
		# current_pose = None
		# return_pose = None
		self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback, queue_size = 1000)
		self.pub_init = rospy.Publisher('/initialization_complete', Bool, queue_size = 3)

	def execute(self, userdata):
		global waypoints, waypointGoingTo, startMovementCheck, waypointsHit, pause, current_pose

		if waypointsHit < len(waypoints):

			if startMovementCheck:
				closest_goal = 0
				closest_distance = 0
				for i in range(0, len(waypoints)):
					goal = self.goal_pose(waypoints[i])
					a, b = goal.target_pose.pose.position.x, goal.target_pose.pose.position.y
					distance = math.sqrt((a-current_pose.pose.position.x)**2 + (b-current_pose.pose.position.y)**2)

					if closest_distance == 0 or distance < closest_distance:
						closest_distance = distance
						closest_goal = i
				startMovementCheck = False
				waypointGoingTo = closest_goal
				print closest_goal
				self.pub_init.publish(True)

			if not pause:

				goal = self.goal_pose(waypoints[waypointGoingTo])
				self.atPoint = False
				
				while not self.atPoint:
					# print 'move'
					if not pause:
						self.move(goal)

				waypointGoingTo += 1
				waypointGoingTo = waypointGoingTo % len(waypoints)
				waypointsHit += 1

			else:
				time.sleep(0.01)
		
		else:
			time.sleep(0.01)

		return 'success'

	def pose_callback(self, data):
		global current_pose
		current_pose = data.pose
		# print current_pose

	def move(self, goal):
		global move_base
		move_base.send_goal(goal)
		cap_time = move_base.wait_for_result(rospy.Duration(60))
		if not cap_time:
			move_base.cancel_goal()
		else:
			if self.is_close(goal):
				print 'close to goal'
				self.atPoint = True
				return #made it close enough, go to next goal
			# state = move_base.get_state()
			# if state == GoalStatus.SUCCEEDED:
			# 	self.atPoint = True
			# 	print 'goal succeeded'
			# 	return

	def is_close(self, goal):
		global current_pose
		a, b = goal.target_pose.pose.position.x, goal.target_pose.pose.position.y
		if current_pose != None:
			d = math.sqrt((a-current_pose.pose.position.x)**2 + (b-current_pose.pose.position.y)**2)
		else:
			d = 0.7
		return d < 0.5 #if close return true otherwise return false

	def goal_pose(self, pose):
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.pose.position.x = pose[0][0]
		goal_pose.target_pose.pose.position.y = pose[0][1]
		goal_pose.target_pose.pose.position.z = pose[0][2]
		goal_pose.target_pose.pose.orientation.x = pose[1][0]
		goal_pose.target_pose.pose.orientation.y = pose[1][1]
		goal_pose.target_pose.pose.orientation.z = pose[1][2]
		goal_pose.target_pose.pose.orientation.w = pose[1][3]
		return goal_pose


if __name__ == '__main__':
	global x_pressed, y_pressed, g_range_ahead, waypoints, waypointGoingTo, startMovementCheck, waypointsHit, pause, move_base, return_pose, current_pose, tag_areas, tag_type
	global soundhandle
	x_pressed = 0
	y_pressed = 0
	g_range_ahead = 0
	waypointGoingTo = 0
	pause = False
	startMovementCheck = True
	waypointsHit = 0
	move_base = False
	return_pose = None
	current_pose = None
	tag_type = ''
	# [[x,y,z], [x,y,z], ..]
	waypoints = [[(4.4884057045, 0.0779395103455, 0.0), (0.0, 0.0, 0.991586551372, 0.129445398293)],
				 [(1.91507136822, 0.765387296677, 0.0), (0.0, 0.0, 0.977148376444, -0.212558345904)],
				 [(0.121204137802, 0.593794822693, 0.0), (0.0, 0.0, 0.997421420608, 0.0717670517161)],
				 [(-2.08100700378, 1.54591023922, 0.0), (0.0, 0.0, 0.997192362885, 0.074882517348)],
				 [(-4.89103031158, 1.8487932682, 0.0), (0.0, 0.0, -0.761332749671, 0.648361353166)],
				 [(-5.474401474, -1.43959307671, 0.0), (0.0, 0.0, -0.119438998482, 0.992841541054)],
				 [(-3.64519500732, -1.72245955467, 0.0), (0.0, 0.0, -0.081600949758, 0.996665081659)],
				 [(-1.02599549294, -2.39323973656, 0.0), (0.0, 0.0, -0.0681311425473, 0.997676374089)],
				 [(1.60786616802, -2.74715876579, 0.0), (0.0, 0.0, -0.102695379529, 0.994712852548)],
				 [(3.8603272438, -2.69449019432, 0.0), (0.0, 0.0, 0.611738147471, 0.791060325721)]]

	tag_areas = []

	soundhandle = SoundClient()

	# soundhandle.say('oaoaoaoaoaoaoaoaoaoaoaoa')
	# soundhandle.say('wawawawawawawawawawawawa')
	# soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')

	scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
	joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
	cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

	init_turn_sub = rospy.Subscriber('dock_start_90', Bool, dock_start_turn_callback, queue_size = 10)
	complete_sub = rospy.Subscriber('docking_done', Bool, dock_complete_callback, queue_size = 10)
	dock_ready = rospy.Publisher('dock_done_90', Bool, queue_size = 5)
	already_found_pub = rospy.Publisher('already_found', Bool, queue_size = 5)

	tag_type_sub = rospy.Subscriber('target_type', String, tag_type_callback, queue_size = 10)

	rospy.init_node('localize')

	sm = StateMachine(outcomes=['success'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	with sm:
		StateMachine.add('Start', Start(), transitions={'success':'Localize'})
		StateMachine.add('Localize', Localize(), transitions={'success':'TraverseMap'})
		StateMachine.add('TraverseMap', TraverseMap(), transitions={'success':'TraverseMap'})

	outcome = sm.execute()




