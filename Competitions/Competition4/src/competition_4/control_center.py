#!/usr/bin/env python
# reference: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_pose/py_pose.html

import rospy, cv2
import numpy as np
from std_msgs.msg import String, Bool
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class ControlCenter:
    def __init__(self):
        self.init_sub = rospy.Subscriber('initialization_complete', Bool, self.init_callback, queue_size=5)  
        self.c_sub = rospy.Subscriber('tag_control', String, self.tag_callback, queue_size=5)  
        self.type_pub = rospy.Publisher('target_type', String, queue_size=5)

        self.docking_complete_sub = rospy.Subscriber('docking_complete', Bool, self.dock_done_callback, queue_size=5)   # from target docker     
        self.dock_done_90_sub = rospy.Subscriber('dock_done_90', Bool, self.done_90_callback, queue_size = 5)
        self.already_found_sub = rospy.Subscriber('already_found', Bool, self.already_found_callback, queue_size = 5)

        self.docking_start_pub = rospy.Publisher('docking_start', Bool, queue_size=5)
        self.docking_done_pub = rospy.Publisher('docking_done', Bool, queue_size=5)
        self.dock_start_90_pub = rospy.Publisher('dock_start_90', Bool, queue_size=5)

        self.init_complete = False
        self.already_found_tag = False
        self.start_90_turn = False
        self.done_90_turn = False
        self.docking_complete = False
        self.docking_init = False
        self.docking = False

        self.soundhandle = SoundClient()

        self.target = ''
        # only change if count hits threshold to minimize error
        self.ua_count = 0
        self.ar_count = 0
        self.old_tag = ''
        self.threshold = 10
        r = rospy.Rate(10)
        while True:
            if self.init_complete:
                # handle docking behaviour
                if self.ua_count >= self.threshold and not self.docking_init:
                    self.start_90_turn = True
                    self.type_pub.publish('UA')

                elif self.ar_count >= self.threshold and not self.docking_init:
                    self.start_90_turn = True
                    self.type_pub.publish('AR')
                    
                if self.start_90_turn and not self.docking_init:
                    self.docking_init = True

                    # print 'play sounds early ', self.target
                    # if self.target == 'UA':
                    #     self.soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')
                    # else:
                    #     self.soundhandle.say('oaoaoaoaoaoaoaoaoaoaoaoa')

                    self.dock_start_90_pub.publish(True)

                if self.done_90_turn and not self.docking:
                    # print 'play sounds late ', self.target
                    self.docking = True
                    # if self.target == 'UA':
                    #     self.soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')
                    # else:
                    #     self.soundhandle.say('oaoaoaoaoaoaoaoaoaoaoaoa')

                    self.docking_start_pub.publish(True)

                if self.docking_complete and self.docking:
                    print 'control center stage 2 docking done'
                    # finished docking behaviour, reset all variables
                    self.already_found_tag = False
                    self.docking = False
                    self.docking_init = False
                    self.docking_complete = False
                    self.done_90_turn = False
                    self.start_90_turn = False
                    self.ar_count = 0
                    self.ua_count = 0
                    self.docking_done_pub.publish(True)

                if self.already_found_tag:
                    self.already_found_tag = False
                    self.docking = False
                    self.docking_init = False
                    self.docking_complete = False
                    self.done_90_turn = False
                    self.start_90_turn = False
                    self.ar_count = 0
                    self.ua_count = 0

            ch = cv2.waitKey(1)
            if ch == 27:
                break
            r.sleep()

    def init_callback(self, data):
        check = data.data
        if check and not self.init_complete:
            self.init_complete = True

    def already_found_callback(self, data):
        check = data.data
        if check:
            self.already_found_tag = True
        else:
            self.already_found_tag = False

    def done_90_callback(self, data):
        check = data.data
        if check:
            self.done_90_turn = True

    def dock_done_callback(self, data):
        check = data.data
        if check:
            print 'control center done docking'
            self.docking_complete = True

    def tag_callback(self, data):
        tag = data.data
        if tag == 'UA':
            # print 'UA'
            self.ar_count = 0
            if self.old_tag != 'UA':
                self.ua_count = 0
            else:
                if self.ua_count < self.threshold:
                    self.ua_count += 1
                else:
                    self.target = tag
        elif tag == 'AR':
            # print 'AR'
            self.ua_count = 0
            if self.old_tag != 'AR':
                self.ar_count = 0
            else:
                if self.ar_count < self.threshold:
                    self.ar_count += 1
                else:
                    self.target = tag
        self.old_tag = tag

if __name__ == "__main__":
    rospy.init_node('control_center')

    # soundhandle.say('oaoaoaoaoaoaoaoaoaoaoaoa')
    # soundhandle.say('wawawawawawawawawawawawa')
    # soundhandle.playWave('/home/hegberg/Music/mlg-airhorn.mp3')
    c = ControlCenter()
    rospy.spin()