#!/usr/bin/env python

import rospy
import actionlib
import sys
import numpy as np
import math
import time
import random
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from std_msgs.msg import String, Bool
from kobuki_msgs.msg import Sound, Led
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty
from sensor_msgs.msg import Image, Joy, LaserScan
from smach import State,StateMachine
import smach_ros
import cv2, cv_bridge
import costmap_2d
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid

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
		move_base.cancel_goal()
		rospy.signal_shutdown('Y Pressed')

	if x_pressed == 1:
		print 'X Pressed'
		# play a sound? Why do we have this?

def vel_callback(msg):
	global velocity, vel_pub
	velocity.x = msg.linear.x
	velocity.y = msg.linear.x
	velocity.z = msg.linear.x
	vel_pub.publish(velocity)

def pose_callback(data):
	global current_pose, pose_pub
	current_pose = data.pose
	pose_pub.publish(data)
	# print current_pose

def ar_callback(msg):
	global ar_found, ar_location, move_base
	# print msg.data
	if len(msg.data) > 0:
		# maybe if we get 10 msgs within a certain time limit set to true?
		ar_found = True
		ar_location = msg.data.split()
		ar_location[0] = float(ar_location[0])
		ar_location[1] = float(ar_location[1])
		move_base.cancel_goal()
		# print 'ar location: ', ar_location
	else:
		ar_found = False

def hider_pose_callback(data):
	global hider_pose
	hider_pose = data.pose
	# print 'hider pose: ', hider_pose

def scan_callback(msg):
	global g_range_ahead
	largeDistance = 100000
	arrayLen = 201
	triPoint = [i for i in range (0,arrayLen)]
	for i in range(0,arrayLen):
		triPoint[i] = msg.ranges[(len(msg.ranges)/2) - ((arrayLen-1)/2) + i]
	g_range_ahead = largeDistance
	for i in range (0, len(triPoint)):
		if (triPoint[i] < g_range_ahead and not math.isnan(triPoint[i])):
			g_range_ahead = triPoint[i]
	if g_range_ahead == largeDistance:
		g_range_ahead = 0

def goal_pose(return_pose):

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

	return goal_pose

def image_callback(msg):
	global centroid, found_hider, move_base

	# bridge = cv_bridge.CvBridge()
	# image = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
	# hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
	# rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
	# # track yellow line
	# # lower_red = np.array([ -10,  240,  240])
	# # upper_red = np.array([ 10, 255, 255])

	# colour_sensitivity = 20
	# sensitivity = 10
	# # lower_red = np.array([40-colour_sensitivity, 255-sensitivity, 255-sensitivity])
	# # upper_red = np.array([40+colour_sensitivity, 255, 255])
	# # filtered = cv2.inRange(hsv, lower_red, upper_red)
	# lower_red = np.array([0, 0, 0])
	# upper_red = np.array([255, sensitivity, sensitivity])
	# filtered = cv2.inRange(hsv, lower_red, upper_red)

	# # lower_red = np.array([ 150,  0,  0])
	# # upper_red = np.array([255, 100, 100])
	# # filtered = cv2.inRange(rgb, lower_red, upper_red)

	# # found_hider = False
	# threshhold_amount = 5000
	# amount_of_pixels = 0
	# bounce_out = False
	# x, y = 0, 0
	# for i in range(0, len(filtered), 10):
	# 	for j in range(0, len(filtered[i]), 10):
	# 		if filtered[len(filtered) - i - 1][j] > 0:
	# 			amount_of_pixels += 1
	# 			if amount_of_pixels >= threshhold_amount:
	# 				found_hider = True
	# 				bounce_out = True
	# 				move_base.cancel_goal()
	# 				x, y = i,j
	# 				print 'found: ', x, ' ', y
	# 				break

	# 	if bounce_out:
	# 		break

	# if not bounce_out: # if didn't break out of loop we didn't find hider
	# 	found_hider = False
	# 	print amount_of_pixels

	# # threshhold_amount = 50

	# # if amount_of_pixels >= threshhold_amount:
	# # 	found_hider = True
	# # else:
	# # 	found_hider = False

	# cv2.imshow('filtered', filtered)
	# # print 'found_hider: ', found_hider, x, y
	# moment = cv2.moments(filtered)

	# if moment['m00']> 0:
	# 	centroid = int(moment['m10']/moment['m00'])

	# # print centroid

	# cv2.waitKey(3)


class Start(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

	def execute(self, userdata):
		global x_pressed
		print 'START'
		while x_pressed == 0:
			time.sleep(0.01)
		global_localize()
		return 'success'

class Localize(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])


	def execute(self, userdata):
		global g_range_ahead, x_pressed, soundhandle
		print 'LOCALIZE'
		rotateTime = rospy.Time.now() + rospy.Duration(10)
		forwardTime = rospy.Time.now() + rospy.Duration(12)
		count = 0

		return 'success'

		while True:
			if g_range_ahead > 1.4 and (rospy.Time.now() < rotateTime):
				twist = Twist()
				twist.angular.z = 0.4
				twist.linear.x = 0
				cmd_vel_pub.publish(twist)
			elif g_range_ahead < 1.51:
				twist = Twist()
				twist.angular.z = 0.3
				twist.linear.x = 0
				cmd_vel_pub.publish(twist)
			elif g_range_ahead > 1.5 and (rospy.Time.now() < forwardTime):
				twist = Twist()
				twist.angular.z = 0
				twist.linear.x = 0.3
				cmd_vel_pub.publish(twist)
			elif count > 0:
				break
			else:
				rotateTime = rospy.Time.now() + rospy.Duration(10)
				forwardTime = rospy.Time.now() + rospy.Duration(12)
				count += 1

		while x_pressed == 0:
			time.sleep(0.01)

		soundhandle.say('Ready or not, here I come')
		# soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')

		return 'success'

class Seek(State):
	def __init__(self):
		State.__init__(self, outcomes=['success', 'failure'])
		global move_base, waypoints, hider_pose, found_hider, height_top, height_bot, width_top, width_bot, cmd_vel_pub, ar_found

	def execute(self, data):
		print 'SEEK'
		while not ar_found:
			# self.move_near_hider()
			print 'Finding new place to check'
			self.move_near_hider()
			continue
			# move around according to cost map

		return 'success'

	def move(self, goal):
		global move_base
		move_base.send_goal(goal)
		cap_time = move_base.wait_for_result(rospy.Duration(60))
		if not cap_time:
			move_base.cancel_goal()
		else:
			state = move_base.get_state()
			if state == actionlib.simple_action_client.GoalStatus.SUCCEEDED:
				self.atPoint = True
				print 'goal succeeded'

				# need to spin here
				rotateTime = rospy.Time.now() + rospy.Duration(15)

				while True:
					if rotateTime < rospy.Time.now() and not found_hider:
						twist = Twist()
						twist.angular.z = 0.8
						twist.linear.x = 0
						cmd_vel_pub.publish(twist)
					else: 
						break

				return

	def move_near_hider(self):
		x = random.uniform(height_bot, height_top)
		y = random.uniform(width_bot, width_top)

		print 'x goal: ', x , 'y goal: ', y
		
		goal_pose = MoveBaseGoal()
		goal_pose.target_pose.header.frame_id = 'map'
		goal_pose.target_pose.header.stamp = rospy.Time.now()
		goal_pose.target_pose.pose.position.x = x
		goal_pose.target_pose.pose.position.y = y
		goal_pose.target_pose.pose.position.z = current_pose.pose.position.z
		goal_pose.target_pose.pose.orientation.w = current_pose.pose.orientation.w
		goal_pose.target_pose.pose.orientation.x = current_pose.pose.orientation.x
		goal_pose.target_pose.pose.orientation.y = current_pose.pose.orientation.y
		goal_pose.target_pose.pose.orientation.z = current_pose.pose.orientation.z

		self.move(goal_pose)

		return

class CatchHider(State):
	def __init__(self):
		State.__init__(self, outcomes=['success', 'failure'])
		global move_base, waypoints, cmd_vel_pub, ar_found, ar_location, found_hider, hider_pose
		self.twist = Twist()

	def execute(self, userdata):
		print 'CATCH HIDER'
		caught = False

		while ar_found and not caught:
			caught = self.pursue(ar_location)

		if caught:
			return 'success'
		else:
			return 'failure'

	def pursue(self, location):
		# lin_kp = 10.0
		ang_kp = 0.005
		# x = lin_kp * (1/float(lin_diff))

		ideal = 315
		c = location[0]  
		ang_diff = ideal - float(c)

		z = ang_diff * ang_kp

		self.twist.angular.z = z
		self.twist.linear.x = 0.7 #should be 0.7 when full speed chasing
		cmd_vel_pub.publish(self.twist)

		# a, b = hider_pose.pose.position.y, hider_pose.pose.position.y
		# d = math.sqrt((a-current_pose.pose.position.x)**2 + (b-current_pose.pose.position.y)**2)
		# if d < 0.5:
		# 	return True
		# else:
		# 	return False

class Win(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

	def execute(self, userdata):
		global win_pub
		twist = Twist()
		twist.linear.x = 0
		twist.angular.z = 0
		cmd_vel_pub.publish(twist)
		win_pub.publish('1')
		print 'WIN'
		return 'success'

class Lose(State):
	def __init__(self):
		State.__init__(self, outcomes=['failure'])

	def execute(self, userdata):
		twist = Twist()
		twist.linear.x = 0
		twist.angular.z = 0
		cmd_vel_pub.publish(twist)
		win_pub('0')
		print 'LOSE'
		return 'failure'

if __name__ == '__main__':
	global x_pressed, y_pressed, current_pose, move_base, cmd_vel_pub, win_pub, soundhandle
	global g_range_ahead, waypoints, pose_pub, velocity, hider_pose, ar_found, ar_location, found_hider
	global height_bot, height_top, width_bot, width_top
	# global soundhandle
	velocity = Point()
	x_pressed = 0
	y_pressed = 0
	g_range_ahead = 0
	current_pose = None
	hider_pose= None

	centroid = None
	found_hider = False

	move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	
	# move_base.wait_for_server(rospy.Duration(60))
	# from comp 4 for testing
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


	height_top = 3.8603272438
	height_bot = -4.89103031158

	width_top = 0.0779395103455
	width_bot = -1.43959307671

	# random.uniform(a,b) a lower, b higher

	soundhandle = SoundClient()

	# # soundhandle.say('oaoaoaoaoaoaoaoaoaoaoaoa')
	# # soundhandle.say('wawawawawawawawawawawawa')
	# # soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')

	rospy.init_node('Seeker')

	scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
	pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback, queue_size = 100)
	joy_sub = rospy.Subscriber('/joy', Joy, joy_callback)
	vel_sub = rospy.Subscriber('/cmd_vel', Twist, vel_callback)
	cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)

	ar_sub = rospy.Subscriber('/ar_location', String, ar_callback)

	# image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback, queue_size = 1, buff_size=2**24)

	pose_pub = rospy.Publisher('/seeker_pose', PoseWithCovarianceStamped, queue_size=100)
	hider_pose_sub = rospy.Subscriber('/hider_pose', PoseWithCovarianceStamped, hider_pose_callback)
	vel_pub = rospy.Publisher('/seeker_velocity', Point, queue_size=100)

	win_pub = rospy.Publisher('/win', String, queue_size = 10)

	# cost_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, cost_map_callback, queue_size = 100)
	# og_cost_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, original_cost_map_callback, queue_size = 100)
	# cost_sub = rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, cost_map_callback, queue_size = 100)
	# og_cost_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, original_cost_map_callback, queue_size = 100)

	# rospy.init_node('Seeker')

	sm = StateMachine(outcomes=['success', 'failure'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_SEEK_ROOT')
	sis.start()

	with sm:
		StateMachine.add('Start', Start(), transitions={'success':'Localize'})
		StateMachine.add('Localize', Localize(), transitions={'success':'Seek'})
		StateMachine.add('Seek', Seek(), transitions={'success':'CatchHider', 'failure':'Lose'})
		StateMachine.add('CatchHider', CatchHider(), transitions={'success':'Win', 'failure':'Seek'})
		StateMachine.add('Win', Win(), transitions={'success':'Win'})
		StateMachine.add('Lose', Lose(), transitions={'failure':'Lose'})

outcome = sm.execute()