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
from sensor_msgs.msg import Joy
from smach import State,StateMachine
import smach_ros
import cv2
import time
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

# def global_localize():
# 	rospy.wait_for_service('global_localization')
# 	localize_service = rospy.ServiceProxy('global_localization', Empty)
# 	try:
# 		res = localize_service()
# 	except:
# 		print("Service did not process request")

# def joy_callback(msg):
# 	global x_pressed
# 	global y_pressed
# 	global move_base

# 	x_pressed = msg.buttons[0]
# 	y_pressed = msg.buttons[2]

# 	if y_pressed == 1:
# 		print 'B Pressed'
# 		move_base.cancel_goal()
# 		rospy.signal_shutdown('Y Pressed')

# 	if x_pressed == 1:
# 		print 'X Pressed'

class Start(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

	def execute(self, userdata):
		global W
		print 'Start'
		while W != 6:
			time.sleep(3)
		# global x_pressed

		# while x_pressed == 0:
		# 	time.sleep(0.01)
		# global_localize()

		return 'success'

class Localize(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

	def execute(self, userdata):
		print 'Localize'
		# global g_range_ahead
		# rotateTime = rospy.Time.now() + rospy.Duration(10)
		# forwardTime = rospy.Time.now() + rospy.Duration(12)
		# count = 0

		# while True:
		# 	if g_range_ahead > 1.4 and (rospy.Time.now() < rotateTime):
		# 		twist = Twist()
		# 		twist.angular.z = 0.4
		# 		twist.linear.x = 0
		# 		cmd_vel_pub.publish(twist)
		# 	elif g_range_ahead < 1.51:
		# 		twist = Twist()
		# 		twist.angular.z = 0.3
		# 		twist.linear.x = 0
		# 		cmd_vel_pub.publish(twist)
		# 	elif g_range_ahead > 1.5 and (rospy.Time.now() < forwardTime):
		# 		twist = Twist()
		# 		twist.angular.z = 0
		# 		twist.linear.x = 0.3
		# 		cmd_vel_pub.publish(twist)
		# 	elif count > 0:
		# 		break
		# 	else:
		# 		rotateTime = rospy.Time.now() + rospy.Duration(10)
		# 		forwardTime = rospy.Time.now() + rospy.Duration(12)
		# 		count += 1
		return 'success'

class Hide(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])
		# global move_base, current_pose
		# move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		# move_base.wait_for_server(rospy.Duration(60))
		# self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback, queue_size = 1000)

	def execute(self, data):
		print 'Hide'
		return 'success'

	# def pose_callback(self, data):
	# 	global current_pose
	# 	current_pose = data.pose
	# 	# print current_pose

	# def move(self, goal):
	# 	global move_base
	# 	move_base.send_goal(goal)
	# 	cap_time = move_base.wait_for_result(rospy.Duration(60))
	# 	if not cap_time:
	# 		move_base.cancel_goal()
	# 	else:
	# 		state = move_base.get_state()
	# 		if state == GoalStatus.SUCCEEDED:
	# 			self.atPoint = True
	# 			print 'goal succeeded'
	# 	return 

class MoveToHomeBase(State):
	def __init__(self):
		State.__init__(self, outcomes=['success', 'failure'])

	def execute(self, data):
		global W
		print 'Home Base'
		W += 1
		print W
		if W % 2 == 0:
			return 'success'
		else:
			return 'failure'
		# win = False
		# if not win:
		# 	return 'fail'
		# return 'success'

class Win(State):
	def __init__(self):
		State.__init__(self, outcomes=['success'])

	def execute(self, data):
		# spin excessively whoop whoop!
		print 'WINNER'
		time.sleep(3)
		return 'success'

class Lose(State):
	def __init__(self):
		State.__init__(self, outcomes=['failure'])

	def execute(self, data):
		# sad music
		print 'LOSER'
		time.sleep(3)
		return 'failure'

if __name__ == '__main__':
	global W
	# global x_pressed, y_pressed
	# # global soundhandle
	# x_pressed = 0
	# y_pressed = 0

	# soundhandle = SoundClient()

	# soundhandle.say('oaoaoaoaoaoaoaoaoaoaoaoa')
	# soundhandle.say('wawawawawawawawawawawawa')
	# soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')

	# joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
	# cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
	W = 0
	rospy.init_node('Hider')

	sm = StateMachine(outcomes=['success', 'failure'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_HIDE_ROOT')
	sis.start()

	with sm:
		StateMachine.add('Start', Start(), transitions={'success':'Localize'})
		StateMachine.add('Localize', Localize(), transitions={'success':'Hide'})
		StateMachine.add('Hide', Hide(), transitions={'success':'MoveToHomeBase'})
		StateMachine.add('MoveToHomeBase', MoveToHomeBase(), transitions={'success':'Win', 'failure':'Lose'})
		StateMachine.add('Win', Win(), transitions={'success':'Win'})
		StateMachine.add('Lose', Lose(), transitions={'failure':'Lose'})

outcome = sm.execute()