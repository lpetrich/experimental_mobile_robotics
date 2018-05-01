#!/usr/bin/env python
# BEGIN ALL
import rospy
import smach_ros
from smach import State,StateMachine
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
  global g_range_ahead
  g_range_ahead = min(msg.ranges)

class Forward(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'])
    global state_change_time

  def execute(self, userdata):
    print 'Forward'
    global state_change_time

    #if (g_range_ahead < 0.8 or rospy.Time.now() > state_change_time):
    while (g_range_ahead > 0.8 and rospy.Time.now() < state_change_time):
      twist = Twist()
      twist.linear.x = 1
      twist.angular.z = 0
      cmd_vel_pub.publish(twist)

    state_change_time = rospy.Time.now() + rospy.Duration(5)
    return 'success'

class Spin(State):
  def __init__(self):
    State.__init__(self, outcomes=['success'])
    global state_change_time

  def execute(self, userdata):
    print 'Spinning'
    global state_change_time

    #if rospy.Time.now() > state_change_time:
    while rospy.Time.now() < state_change_time:
      twist = Twist()
      twist.angular.z = 1
      twist.linear.x = 0
      cmd_vel_pub.publish(twist)

    state_change_time = rospy.Time.now() + rospy.Duration(30)
    return 'success'

if __name__ == '__main__':

  g_range_ahead = 1 # anything to start
  scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  rospy.init_node('wanderSM')
  state_change_time = rospy.Time.now()
  #driving_forward = True
  rate = rospy.Rate(10)

  sm = StateMachine(outcomes=['success'])

  sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
  sis.start()

  with sm:
    StateMachine.add('Forward', Forward(), transitions={'success':'Spin'})
    StateMachine.add('Spin', Spin(), transitions={'success':'Forward'})


  outcome = sm.execute()