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
from sensor_msgs.msg import Joy, LaserScan
from smach import State,StateMachine
import smach_ros
import cv2
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

def ar_callback(msg):
	global ar_found, ar_location
	# print msg.data
	if len(msg.data) > 0:
		# maybe if we get 10 msgs within a certain time limit set to true?
		ar_found = True
		ar_location = msg.data.split()
		ar_location[0] = float(ar_location[0])
		ar_location[1] = float(ar_location[1])
		# print 'ar location: ', ar_location
	else:
		ar_found = False

def pose_callback(data):
	global current_pose, pose_pub
	current_pose = data.pose
	pose_pub.publish(data)
	# print current_pose

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

def original_cost_map_callback(data):
	global costmap_og, map_height, map_width
	# print data
	# print('og costmap')
	print(len(data.data))
	# print('----------------------------------------------------------------------------------------------------')
	costmap_og = data

def cost_map_callback(data):
	global costmap, map_width, map_height, map_initialized
# 	header: 
#   seq: 5
#   stamp: 
#     secs: 1523406448
#     nsecs: 109183886
#   frame_id: odom
# x: 24
# y: 6
# width: 56
# height: 56
# data: [0, 
	# print (data)
	print('costmap update: x: ', data.x, ' y:', data.y, ' height: ', data.height, ' width: ', data.width)
	# print(len(data.data))
	# print('___________________________________________________________________________________________________________')
	if not map_initialized:
		map_width = data.width
		map_height = data.height
		map_initialized = True
	costmap = data

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
		global g_range_ahead
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

		return 'success'

class Seek(State):
	def __init__(self):
		State.__init__(self, outcomes=['success', 'failure'])
		global move_base, ar_found, ar_location, waypoints, map_height, map_width, costmap

	def execute(self, data):
		print 'SEEK'
		while not ar_found:
			self.find_lowest_cost_area()
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
				return

	def find_lowest_cost_area(self):
		# current position x, y -> width*y + x = current index in list

		temp_costmap = costmap

		# top
		top_sum = 0
		if (temp_costmap.y + 15) < temp_costmap.height and (temp_costmap.x + 5) < temp_costmap.width and (temp_costmap.x - 5) > 0:
			for i in range(10, 15):
				for j in range(-5, 5):
					if temp_costmap.data[(temp_costmap.y * temp_costmap.width) + (i * temp_costmap.width) + (temp_costmap.x + j)] >= 0:
						top_sum += temp_costmap.data[(temp_costmap.y * temp_costmap.width) + (i * temp_costmap.width) + (temp_costmap.x + j)]
		
		print 'top sum: ', top_sum
		
		# bot
		bot_sum = 0
		if (temp_costmap.y - 15) > 0 and (temp_costmap.x + 5) < temp_costmap.width and (temp_costmap.x - 5) > 0:
			for i in range(-10, -15):
				for j in range(-5, 5):
					if temp_costmap.data[(temp_costmap.y * temp_costmap.width) + (i * temp_costmap.width) + (temp_costmap.x + j)] >= 0:
						bot_sum += temp_costmap.data[(temp_costmap.y * temp_costmap.width) + (i * temp_costmap.width) + (temp_costmap.x + j)]

		print 'bot sum: ', bot_sum

class CatchHider(State):
	def __init__(self):
		State.__init__(self, outcomes=['success', 'failure'])
		global move_base, ar_found, ar_location, waypoints, cmd_vel_pub
		self.twist = Twist()

	def execute(self, userdata):
		print 'CATCH HIDER'
		caught = False

		while ar_found and not caught:
			self.pursue(ar_location)

			

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
		# if ang_diff < -10:
		#     z = -0.1
		# elif ang_diff > 10:
		#     z = 0.1
		# else:
		#     z = 0
		z = ang_diff * ang_kp

		# if x > 0.4:
		#     x = 0.4
		# # print x
		# if x < 0.03:
		#     return True

		# print "z: ", z

		self.twist.angular.z = z
		self.twist.linear.x = 0.0 #should be 0.7 when full speed chasing
		cmd_vel_pub.publish(self.twist)

class Win(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

	def execute(self, userdata):
		print 'WIN'
		return 'success'

class Lose(State):
	def __init__(self):
		State.__init__(self, outcomes=['failure'])

	def execute(self, userdata):
		print 'LOSE'
		return 'failure'

if __name__ == '__main__':
	global x_pressed, y_pressed, ar_found, ar_location, current_pose, move_base, cmd_vel_pub
	global g_range_ahead, waypoints, pose_pub, velocity, map_height, map_width, map_initialized
	# global soundhandle
	velocity = Point()
	x_pressed = 0
	y_pressed = 0
	g_range_ahead = 0
	ar_found = False
	ar_location = []
	current_pose = None
	move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	map_initialized = False
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
	# soundhandle = SoundClient()

	# # soundhandle.say('oaoaoaoaoaoaoaoaoaoaoaoa')
	# # soundhandle.say('wawawawawawawawawawawawa')
	# # soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')
	scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)
	pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback, queue_size = 100)
	ar_sub = rospy.Subscriber('/ar_location', String, ar_callback)
	joy_sub = rospy.Subscriber('/joy', Joy, joy_callback)
	vel_sub = rospy.Subscriber('/cmd_vel', Twist, vel_callback)
	cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1)
	pose_pub = rospy.Publisher('/seeker_pose', PoseWithCovarianceStamped, queue_size=100)
	vel_pub = rospy.Publisher('/seeker_velocity', Point, queue_size=100)

	# cost_sub = rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, cost_map_callback, queue_size = 100)
	# og_cost_sub = rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, original_cost_map_callback, queue_size = 100)
	cost_sub = rospy.Subscriber('/move_base/local_costmap/costmap_updates', OccupancyGridUpdate, cost_map_callback, queue_size = 100)
	og_cost_sub = rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, original_cost_map_callback, queue_size = 100)

	rospy.init_node('Seeker')

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