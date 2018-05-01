#!/usr/bin/env python

import roslib
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Joy
from tf import TransformListener
from visualization_msgs.msg import Marker
from math import radians, pi, sqrt
import time

def joy_callback(msg):
  global x_pressed
  global y_pressed

  #print msg.buttons[0]

  x_pressed = msg.buttons[0]
  y_pressed = msg.buttons[2]

  if y_pressed == 1:
    rospy.signal_shutdown('Y Pressed')

class StartRace:
    def __init__(self):
        rospy.init_node('racing_turtle', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.T = TransformListener()
        self.sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback, queue_size = 1000)
        self.sub2 = rospy.Subscriber('/waypoint', PoseStamped, self.goal_callback, queue_size = 1000)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        # self.marker_pub = rospy.Publisher('pylon_markers', Marker, queue_size = 1) #WHY DOES THE WORD PYLON LOOK SO WEIRD
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))
        # self.markers = self.init_marker()
        self.goals = []
        self.current_pose = None
        self.initial_pose = None
        self.counter = 0
        self.init = False
        self.start = False
        self.finished = False
        self.atPoint = False
        # self.waypoints = [None] * 8
        self.quaternions = []
        euler_angles = (pi/2, pi, 3*pi/2, 0)
        for angle in euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            self.quaternions.append(Quaternion(*q_angle))
       
        # print self.quaternions
        self.init_position()
        rate = rospy.Rate(1.0)
        firstPoint = True
        while not rospy.is_shutdown() and not self.finished:
            if self.start:
                print 'GOALS: ', self.goals
                for i in range((len(self.goals) * 2) + 2):
                    self.atPoint = False
                    waypoint = self.set_waypoint(i % 6)
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose = waypoint
                    while not self.atPoint:
                        print 'move'
                        self.move(goal)
                    if firstPoint:
                        while x_pressed == 0:
                            time.sleep(0.01)
                        firstPoint = False

                    # rate.sleep()
                self.finished = True
                self.start = False

    def init_position(self):
        print '>>> click the 2D Pose Estimate button in RViz to set the robots initial pose'
        msg = rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.initial_pose = msg.pose
        print 'initial ', self.initial_pose, '\n'

    def set_waypoint(self, i):
        waypoint = Pose(Point(self.goals[i].position.x, self.goals[i].position.y, self.goals[i].position.z), self.goals[i].orientation)
        return waypoint

    def pose_callback(self, data):
        self.current_pose = data.pose
        # print self.current_pose.pose.position.x
           
    def goal_callback(self, data):
        if self.counter < 6:
            self.goals.append(data.pose)
            # print self.goals
            self.counter += 1
            if self.counter == 6:
                self.start = True
                print 'START IS TRUE'

    def is_close(self, goal):
        a, b = goal.target_pose.pose.position.x, goal.target_pose.pose.position.y
        if self.current_pose != None:
            d = sqrt((a-self.current_pose.pose.position.x)**2 + (b-self.current_pose.pose.position.y)**2)
        else:
            d = 0.7
        return d < 0.5 #if close return true otherwise return false
          
    def move(self, goal):
        self.move_base.send_goal(goal)
        # self.move_base.wait_for_result(rospy.Duration(1.0))
        # if self.is_close(goal):
        #     print 'close to goal'
        #     self.atPoint = True
        #     return #made it close enough, go to next goal
        # state = self.move_base.get_state()
        # if state == GoalStatus.SUCCEEDED:
        #     self.atPoint = True
        #     print 'goal succeeded'
        #     return
        cap_time = self.move_base.wait_for_result(rospy.Duration(60))
        if not cap_time:
            self.move_base.cancel_goal()
            print 'timed out'
        else:
            if self.is_close(goal):
                print 'close to goal'
                self.atPoint = True
                return #made it close enough, go to next goal
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                self.atPoint = True
                print 'goal succeeded'
                return
                                 
    # def init_marker(self):
    #     # for visualization in rviz
    #     marker_scale = 0.2
    #     marker_lifetime = 0
    #     marker_ns = 'waypoint'
    #     marker_id = 0
    #     marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}  

    #     m = Marker()
    #     m.ns = marker_ns
    #     m.id = marker_id
    #     m.type = Marker.SPHERE_LIST
    #     m.action = Marker.ADD
    #     m.lifetime = rospy.Duration(marker_lifetime)
    #     m.scale.x = marker_scale
    #     m.scale.y = marker_scale
    #     m.color.r = marker_color['r']
    #     m.color.g = marker_color['g']
    #     m.color.b = marker_color['b']
    #     m.color.a = marker_color['a']
    #     m.header.frame_id = 'map'
    #     m.header.stamp = rospy.Time.now()
    #     m.points = list()        
    #     return m

    def shutdown(self):
        print 'stop'
        self.move_base.cancel_goal()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(2)

if __name__ == '__main__':
    x_pressed = 0
    joy_sub = rospy.Subscriber('joy', Joy, joy_callback)
    try:
        StartRace()
    except rospy.ROSInterruptException:
        print 'error'