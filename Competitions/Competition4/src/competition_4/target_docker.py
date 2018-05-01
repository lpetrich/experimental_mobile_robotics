#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from std_msgs.msg import String, UInt32, Bool
from sensor_msgs.msg import CompressedImage, LaserScan, CameraInfo
from geometry_msgs.msg import Twist
import math
import time
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class TargetDocker:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.twist = Twist()

        self.start_docking = False
        self.init_target = False
        self.get_wall_distance = False
        self.target_acquired = ''
        self.current_tag = 'UA'
        self.target_location = []
        self.wall_distance = 1

        self.start_docking_sub = rospy.Subscriber('docking_start', Bool, self.dock_callback, queue_size = 10)
        self.location_sub = rospy.Subscriber('target_location', String, self.target_location_cb)
        self.type_sub = rospy.Subscriber('target_type', String, self.target_type_cb)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.dock_pub = rospy.Publisher('docking_complete', Bool, queue_size = 1)

        self.soundhandle = SoundClient()

    # soundhandle.say('oaoaoaoaoaoaoaoaoaoaoaoa')
    # soundhandle.say('wawawawawawawawawawawawa')
    # soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')

        r = rospy.Rate(10)
        while True:
            if self.start_docking and self.target_acquired != None and len(self.target_location) > 0:
                # print 'start docking'
                check = False
                if self.current_tag == 'AR':
                    self.get_wall_distance = True
                    while not check:
                        check = self.navigate_ar()
                        r.sleep()
                else:
                    while not check:
                        target = self.target_location
                        centroid = self.calc_centroid(target)
                        if centroid != None:
                            diff = target[2][0] - target[0][0] # this probably isnt right
                            if diff != 0:
                                check = self.navigate(centroid, diff, target)
                        r.sleep()
                if check:
                    self.dock_pub.publish(True)
                    self.get_wall_distance = False
                    self.wall_distance = 1
                    self.start_docking = False
                    self.init_target = False
                    self.target_acquired = None
                    print 'target_docker DONE DOCKING'
                        # MAKE BEEPING NOISE
            r.sleep()

    def scan_cb(self, data):
        if self.get_wall_distance:
            mid = len(data.ranges) / 2
            vals = data.ranges[mid - 20:mid + 20]
            val_mean = 0
            for i in vals:
                if not math.isnan(i):
                    val_mean += i
            # distance to wall
            self.wall_distance = val_mean / len(vals)

    def dock_callback(self, data):
        check = data.data
        if check and not self.start_docking:
            self.start_docking = True
            self.current_tag = self.target_acquired

    def target_location_cb(self, data):
        msg = data.data.split()
        del self.target_location[:]
        for i in range(0, len(msg), 2):
            self.target_location.append([int(msg[i]), int(msg[i + 1])])

    def target_type_cb(self, data):
        if not self.init_target:
            self.target_acquired = data.data
            self.init_target = True

    def calc_centroid(self, corners):
        # corners =  np.array(corners, dtype=np.float64)
        # P = corners.tolist()
        P = corners
        # s = np.vstack([P[0],P[2],P[1],P[3]])
        s = np.vstack([P[0],P[1],P[2],P[3]])
        H = np.hstack((s, np.ones((4, 1))))
        l1 = np.cross(H[0], H[2])           
        l2 = np.cross(H[1], H[3])         
        x, y, z = np.cross(l1, l2)
        return [int(x / z), int(y / z)]

    def navigate(self, centroid, lin_diff, target):
        lin_kp = 10.0
        ang_kp = 0.0005
        x = lin_kp * (1/float(lin_diff))

        ideal = 315
        c = centroid[0]        
        ang_diff = ideal - c
        if ang_diff < -10:
            z = -0.1
        elif ang_diff > 10:
            z = 0.1
        else:
            z = 0

        if x > 0.4:
            x = 0.4
        # print x
        if x < 0.03:
            return True

        self.twist.angular.z = z
        self.twist.linear.x = x
        self.cmd_vel_pub.publish(self.twist)
        return False

    def navigate_ar(self):
        target = self.target_location
        u = (target[0][0] + target[1][0] + target[2][0] + target[3][0]) / 4
        ideal = 315
        # y = (target[0][1] + target[1][1] + target[2][1] + target[3][1]) / 4
        print 'distance to wall: ', self.wall_distance
        lin_kp = 1
        x = (lin_kp * self.wall_distance)/5
        print 'x: ', x  

        ang_diff = ideal - u
        if ang_diff < -10:
            z = -0.1
        elif ang_diff > 10:
            z = 0.1
        else:
            z = 0    
        # WHAT SHOULD THE CUTOFF VALUE BE?!?!
        # CHRISTOPHER ADJUST
        if self.wall_distance < 0.2:
            print 'CLOSE TO AR TAG'
            self.twist.angular.z = z
            self.twist.linear.x = x
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(0.1)
            return True

        self.twist.angular.z = z
        self.twist.linear.x = x
        self.cmd_vel_pub.publish(self.twist)  
        return False

if __name__ == "__main__":
    rospy.init_node('target_docker')
    td = TargetDocker()
    rospy.spin()
