#!/usr/bin/env python
# BEGIN ALL
import rospy
import smach_ros
from smach import State,StateMachine
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Joy
import math
import random
import time


def joy_callback(msg):
  global x_pressed
  global y_pressed

  #print msg.buttons[0]

  x_pressed = msg.buttons[0]
  y_pressed = msg.buttons[2]

  if y_pressed == 1:
    rospy.signal_shutdown('Y Pressed')

def scan_callback(msg):
  global g_range_ahead

  largeDistance = 100

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

def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
  step = ramp_rate*(t_now - t_prev).to_sec()

  sign = 1.0 if (v_target>v_prev) else -1.0

  error = math.fabs(v_target - v_prev)

  if error < step:
    return v_target
  else:
    return v_prev + sign * step


class Start(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys = ['state_change_time_in'], output_keys = ['state_change_time_out'])
    #global state_change_time

  def execute(self, userdata):
    print 'Start'
    #print userdata.state_change_time_in
    print g_range_ahead

    global x_pressed

    while x_pressed == 0:
      time.sleep(0.01)

    userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(30)
    return 'success'

class Forward(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys = ['state_change_time_in'], output_keys = ['state_change_time_out'])
    #global state_change_time

  def execute(self, userdata):
    print 'Forward'
    #print userdata.state_change_time_in
    print g_range_ahead

    global t_prev, x_last, z_last, ramp

    x_target = 0.8
    z_target = 0.8

    x_last = 0
    z_last = 0

    t_prev = rospy.Time.now()

    #if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
    while (g_range_ahead > 0.8 and (rospy.Time.now() < userdata.state_change_time_in)):
      print(g_range_ahead)
      twist = Twist()
      t_now = rospy.Time.now()
      #twist.linear.x = 1.0
      #twist.angular.z = 0.5
      twist.linear.x = ramped_vel(x_last, x_target, t_prev, t_now, ramp)
      x_last = twist.linear.x
      twist.angular.z = ramped_vel(z_last, z_target, t_prev, t_now, ramp + 1)
      z_last = twist.angular.z
      cmd_vel_pub.publish(twist)

      t_prev = t_now

    userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(3)
    return 'success'

class Spin(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys = ['state_change_time_in'], output_keys = ['state_change_time_out'])
    #global state_change_time

  def execute(self, userdata):
    print 'Spinning'
    #print userdata.state_change_time_in

    global additionalTime
    additionalTime += 1

    #stop moving
    if g_range_ahead < 0.8:
      twist = Twist()
      twist.angular.z = 0
      twist.linear.x = 0
      cmd_vel_pub.publish(twist)

      #if rospy.Time.now() > state_change_time:
      #while rospy.Time.now() < userdata.state_change_time_in:
      print 'find open'
      while g_range_ahead < 0.8:
        twist = Twist()
        twist.angular.z = 1
        twist.linear.x = 0
        cmd_vel_pub.publish(twist)


      tempTime = rospy.Time.now() + rospy.Duration(additionalTime)

      #while (rospy.Time.now() < tempTime and g_range_ahead >= 0.5): 
      print 'additional time'
      while (rospy.Time.now() < tempTime):
        cmd_vel_pub.publish(twist)

      tooFar = False

      if g_range_ahead < 0.8:
        twist.angular.z = -1
        twist.linear.x = 0

        additionalTime = 0

        tooFar = True

      if tooFar:
        print 'find open backwards'
        while g_range_ahead < 0.7: 
          cmd_vel_pub.publish(twist)

    userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(30)
    return 'success'

class Circle_SetUp(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys = ['state_change_time_in'], output_keys = ['state_change_time_out'])
    #global state_change_time

  def execute(self, userdata):
    print 'Circle Set Up'
    #print userdata.state_change_time_in

    #if rospy.Time.now() > state_change_time:
    #while rospy.Time.now() < userdata.state_change_time_in:
    print 'find open'
    #print g_range_ahead

    #stop moving
    twist = Twist()
    twist.angular.z = 0
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)

    while g_range_ahead < 1.0:
      twist = Twist()
      twist.angular.z = 1
      twist.linear.x = 0
      cmd_vel_pub.publish(twist)


    #tempTime = rospy.Time.now() + rospy.Duration(1)

    #while (rospy.Time.now() < tempTime):
      #twist = Twist()
      #twist.angular.z = 1
      #twist.linear.x = 0
      #cmd_vel_pub.publish(twist)
      #print g_range_ahead

    print g_range_ahead

    userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(15)
    return 'success'

class Circle(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'], input_keys = ['state_change_time_in'], output_keys = ['state_change_time_out'])
    #global state_change_time

  def execute(self, userdata):
    print 'Circle'
    #print userdata.state_change_time_in
    print g_range_ahead

    global t_prev, x_last, z_last, ramp

    x_target = 0.8
    z_target = 3.0

    x_last = 0
    z_last = 0

    t_prev = rospy.Time.now()

    time = rospy.Time.now() + rospy.Duration(5)

    #if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
    while (g_range_ahead > 0.7 and (rospy.Time.now() < time)):
      twist = Twist()
      #twist.linear.x = 2.0
      #twist.angular.z = 3.0
      #cmd_vel_pub.publish(twist)

      t_now = rospy.Time.now()
      twist.linear.x = ramped_vel(x_last, x_target, t_prev, t_now, ramp)
      x_last = twist.linear.x
      twist.angular.z = ramped_vel(z_last, z_target, t_prev, t_now, ramp + 1)
      z_last = twist.angular.z
      cmd_vel_pub.publish(twist)
      t_prev = t_now

    t_now = rospy.Time.now()
    twist = Twist()
    x_target = 0.8
    z_target = -0.0
    cmd_vel_pub.publish(twist)
    t_prev = t_now
    x_target = 0.8
    z_target = -3.0

    time = rospy.Time.now() + rospy.Duration(5)

    while (g_range_ahead > 0.7 and (rospy.Time.now() < time)):
      twist = Twist()
      #twist.linear.x = 2.0
      #twist.angular.z = 3.0
      #cmd_vel_pub.publish(twist)

      t_now = rospy.Time.now()
      twist.linear.x = ramped_vel(x_last, x_target, t_prev, t_now, ramp)
      x_last = twist.linear.x
      twist.angular.z = ramped_vel(z_last, z_target, t_prev, t_now, ramp + 1)
      z_last = twist.angular.z
      cmd_vel_pub.publish(twist)
      t_prev = t_now

    #if g_range_ahead > 0.5:
      #print 'not range'

    #if rospy.Time.now() < userdata.state_change_time_in:
      #print 'not time'
      #print g_range_ahead

    userdata.state_change_time_out = rospy.Time.now() + rospy.Duration(5)
    return 'success'

if __name__ == '__main__':

  g_range_ahead = 1 # anything to start
  x_pressed = 0
  y_pressed = 0

  ramp = 1.0



  scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
  joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  rospy.init_node('wanderSM')
  #state_change_time = rospy.Time.now()
  #driving_forward = True
  rate = rospy.Rate(30)

  additionalTime = 0

  sm = StateMachine(outcomes=['success'])
  sm.userdata.state_change_time = rospy.Time.now()

  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  #while x_pressed == 0:
    #time.sleep(0.1)

  t_prev = rospy.Time.now()


  with sm:
    #StateMachine.add('Circle', Circle(), transitions={'success':'Spin'}, remapping={'state_change_time_in':'state_change_time', 'state_change_time_out':'state_change_time'})
    StateMachine.add('Start', Start(), transitions={'success':'Forward'}, remapping={'state_change_time_in':'state_change_time', 'state_change_time_out':'state_change_time'})
    StateMachine.add('Forward', Forward(), transitions={'success':'Circle_SetUp'}, remapping={'state_change_time_in':'state_change_time', 'state_change_time_out':'state_change_time'})
    StateMachine.add('Spin', Spin(), transitions={'success':'Forward'}, remapping={'state_change_time_in':'state_change_time', 'state_change_time_out':'state_change_time'})
    StateMachine.add('Circle_SetUp', Circle_SetUp(), transitions={'success':'Circle'}, remapping={'state_change_time_in':'state_change_time', 'state_change_time_out':'state_change_time'})
    StateMachine.add('Circle', Circle(), transitions={'success':'Spin'}, remapping={'state_change_time_in':'state_change_time', 'state_change_time_out':'state_change_time'})
    

  outcome = sm.execute()
