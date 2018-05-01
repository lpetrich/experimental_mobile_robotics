#!/usr/bin/env python
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import Twist
from smach import State,StateMachine
import smach_ros

def joy_callback(msg):
	global x_pressed
	global y_pressed

	#print msg.buttons[0]

	x_pressed = msg.buttons[0]
	y_pressed = msg.buttons[2]

	if y_pressed == 1:
		rospy.signal_shutdown('Y Pressed')

class Start(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'])
    #global state_change_time
    cv2.namedWindow("window", 1)

  def execute(self, userdata):
    print 'Start'
    #print userdata.state_change_time_in

    global x_pressed
    #global M, image, w, cmd_vel_pub

    while x_pressed == 0:
      time.sleep(0.01)

    #cv2.imshow("window", image)

    #time.sleep(1.01)

    return 'success'



class Follower(State):
	def __init__(self):

		State.__init__(self, outcomes=['success'])

		self.bridge = cv_bridge.CvBridge()
		cv2.namedWindow("window", 1)
		self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
		self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
		self.twist = Twist()
		#self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

	def execute(self, userdata):
		ang_kp = 0.5
		ang_kd = 0.015	
		centroid_left = numpy.array([0, 0])
		centroid_right = numpy.array([0, 0])
		bot_track_center = numpy.array([0, 0])
		top_track_center = numpy.array([0, 0])
		q = numpy.array([0, 0])
		prev_time = rospy.get_time()
		lin_error = 0
		ang_error = 0
		min_lin = 0.3
		max_lin = 0.8
		lin = 0.2
		x_mid_point = 250

		if self.M_bot_right['m00'] > 0:
			centroid_right[0] = int(self.M_bot_right['m10']/self.M_bot_right['m00']) # x position
			centroid_right[1] = int(self.M_bot_right['m01']/self.M_bot_right['m00']) # y position

			if self.M_bot_left['m00'] > 0:
				centroid_left[0] = int(self.M_bot_left['m10']/self.M_bot_left['m00']) # x position
				centroid_left[1] = int(self.M_bot_left['m01']/self.M_bot_left['m00']) # y position
				x_mid_point = (centroid_right[0] - centroid_left[0]) / 2
				bot_track_center[0] = centroid_left[0] + x_mid_point # x bottom center
				bot_track_center[1] = (centroid_left[1] + centroid_right[1]) / 2 # y bottom center


			else: # bot sees o left so need hard left turn
				bot_track_center[0] = centroid_right[0] - x_mid_point # x bottom center
				bot_track_center[1] = centroid_right[1] # y bottom center


			ang_error = bot_track_center[0] - (self.w / 2)

			cv2.circle(self.image, (bot_track_center[0], bot_track_center[1]) , 10, (255,0,0), -1)   
			cv2.circle(self.image, (centroid_left[0], centroid_left[1]), 10, (0,0,255), -1)   
			cv2.circle(self.image, (centroid_right[0], centroid_right[1]), 10, (0,0,255), -1)   

			if self.M_top_right['m00'] > 0:
				# centroid_left[0] = int(self.M_top_left['m10']/self.M_top_left['m00']) # x position
				# centroid_left[1] = int(self.M_top_left['m01']/self.M_top_left['m00']) # y position
				centroid_right[0] = int(self.M_top_right['m10']/self.M_top_right['m00']) # x position
				centroid_right[1] = int(self.M_top_right['m01']/self.M_top_right['m00']) # y position
				top_track_center[0] = centroid_right[0] - x_mid_point + 50
				top_track_center[1] = centroid_right[1]
				# lin_error = top_track_center[0] - (self.w / 2)
				# print 'top center error ', lin_error
				cv2.circle(self.image, (top_track_center[0], top_track_center[1]), 10, (0,255,0), -1)   
				# cv2.circle(self.image, (centroid_left[0], centroid_left[1]), 10, (0,0,255), -1)   
				cv2.circle(self.image, (centroid_right[0], centroid_right[1]), 10, (0,0,255), -1)  

				# dist = numpy.linalg.norm(bot_track_center - top_track_center) 
				c_ang = numpy.dot(bot_track_center, top_track_center)
				s_ang = numpy.linalg.norm(numpy.cross(bot_track_center, top_track_center))
				angle = numpy.rad2deg(numpy.arctan2(s_ang, c_ang))
				print "angle ", angle
				lin = numpy.interp(angle, [0, 40], [min_lin, max_lin])
				print "lin " , lin

			# check change in time for derivative calculation
			time.sleep(0.005)
			current_time = rospy.get_time()
			delta_t = current_time - prev_time
			prev_time = current_time

			ang_P = ang_kp * ang_error
			ang_D = ang_kd * (ang_error / delta_t)
			ang = -float(ang_P + ang_D) / 100

		 	self.twist.linear.x = lin 
			self.twist.angular.z = ang
			self.cmd_vel_pub.publish(self.twist)

		else:
			self.twist.linear.x = 0.2
			self.twist.angular.z = -0.2
			self.cmd_vel_pub.publish(self.twist)
		# cv2.imshow("window", self.image)
		cv2.waitKey(3)

		return 'success'

	def image_callback(self, msg):
		self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
		# track yellow line
		lower_yellow = numpy.array([ 10,  10,  10])
		upper_yellow = numpy.array([255, 255, 250])

		test_sensitivity = 60
		sensitivity = 35
		lower_white = numpy.array([0,0,255-sensitivity])
		upper_white = numpy.array([255,sensitivity,255])
		# lower_white = numpy.array([ 0,  0,  200])
		# upper_white = numpy.array([255, 0, 255])
		bot_left = cv2.inRange(hsv, lower_white, upper_white)
		bot_right = cv2.inRange(hsv, lower_white, upper_white)
		top_left = cv2.inRange(hsv, lower_white, upper_white)
		top_right = cv2.inRange(hsv, lower_white, upper_white)

		cv2.imshow('bl', bot_left)


		h, self.w, d = self.image.shape

		h_mid = h / 2
		w_mid = self.w / 2

		adjustment = 100

		top_left[h/2:h, 0:self.w] = 0
		top_left[0:h/2, (self.w/2)-adjustment:self.w] = 0

		top_right[h/2:h, 0:self.w] = 0
		top_right[0:h/2, 0:(self.w/2)+adjustment] = 0

		bot_left[h/2:h, self.w/2:self.w] = 0
		bot_left[0:h/2, 0:self.w] = 0

		bot_right[h/2:h, 0:self.w/2] = 0
		bot_right[0:h/2, 0:self.w] = 0

		self.M_bot_left = cv2.moments(bot_left)
		self.M_bot_right = cv2.moments(bot_right)
		self.M_top_left = cv2.moments(top_left)
		self.M_top_right = cv2.moments(top_right)
		#cv2.imshow('bl', bot_left)
		#cv2.imshow('br', bot_right)
		#cv2.imshow('tl', top_left)
		#cv2.imshow('tr', top_right)

def main():

	global x_pressed, y_pressed, cmd_vel_pub
	x_pressed = 0
	y_pressed = 0

	joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
	#image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)
	cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

	rospy.init_node('Follower')

	sm = StateMachine(outcomes=['success'])

	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()


	with sm:
		StateMachine.add('Start', Start(), transitions={'success':'Follower'})
		StateMachine.add('Follower', Follower(), transitions={'success':'Follower'})

	outcome = sm.execute()

	#rospy.init_node('follower')
	#follower = Follower()
	#rospy.spin()
	# END ALL

if __name__ == '__main__':

	main()