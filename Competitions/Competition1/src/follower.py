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

  x_pressed = msg.buttons[0]
  y_pressed = msg.buttons[2]

  if y_pressed == 1:
    rospy.signal_shutdown('Y Pressed')

def scan_callback(msg):
  global evaders_pos
  global drift

  largeDistance = 100 # arbitrarly largeS

  triPoint = [i for i in range (0,121)]

  for i in range(0,121):
    triPoint[i] = msg.ranges[(len(msg.ranges)/2) - 60 + i]


  evaders_pos = largeDistance
  for i in range (0, len(triPoint)):
    if (triPoint[i] < evaders_pos and not math.isnan(triPoint[i])):
      evaders_pos = triPoint[i]
      drift = i

  if evaders_pos == largeDistance:
    evaders_pos = 0


class Follower(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'])

  def execute(self, userdata):
    print 'Start'
    print 'evader position ', evaders_pos
    print 'drift ', drift

    global x_pressed

    while x_pressed == 0:
      time.sleep(0.01)

    return 'success'

class Follow(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'])

  def execute(self, userdata):
    print 'Follow'
    print 'evader position ', evaders_pos
    print 'drift ', drift
    lin = 0
    ang = 0 

    start_time = rospy.Time.now()
    twist = Twist()
    # twist.linear.x = 1.0
    #twist.linear.z = -2.0

    while (rospy.Time.now() - start_time) < rospy.Duration(60)):
      print 'evader position ', evaders_pos
      print 'drift ', drift
      if evaders_pos > 0.82: # evader too far away
        lin += 0.1
      else if evaders_pos < 0.78 # evader too close
        lin -= 0.1
      if drift < 50: # evader to the left
        ang -= 0.1
      else if drift > 70:
        ang += 0.1

      twist = Twist()
      twist.linear.x = lin
      twist.angular.z = ang 
      cmd_vel_pub.publish(twist)

    #if evaders_pos > 0.5:
      #print 'not range'

    return 'success'

class Winner(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'])
    #global state_change_time

  def execute(self, userdata):
    print 'Spinning'

    global additionalTime
    additionalTime += 1


    twist = Twist()
    twist.angular.z = 1
    twist.linear.x = 0

    while True:
      twist = Twist()
      twist.angular.z = 1
      twist.linear.x = 0
      cmd_vel_pub.publish(twist)

    return 'success'

if __name__ == '__main__':

  evaders_pos = 1 # anything to start
  x_pressed = 0
  y_pressed = 0
  scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
  joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  rospy.init_node('Follower')
  
  rate = rospy.Rate(30)

  sm = StateMachine(outcomes=['success'])
  sm.userdata.state_change_time = rospy.Time.now()

  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  #while x_pressed == 0:
    #time.sleep(0.1)

  with sm:
    StateMachine.add('Follower', Start(), transitions={'success':'Follow'})
    StateMachine.add('Follow', Forward(), transitions={'success':'Winner'})
    StateMachine.add('Winner', Winner(), transitions={'success':'Winner'})

  outcome = sm.execute()
