#!/usr/bin/env python

# Resource: https://github.com/npinto/opencv/tree/master/samples/python2
# Laura Petrich and Chris Hegberg
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2

class TagDetector:
    def __init__(self):
        FLANN_INDEX_KDTREE = 1
        FLANN_INDEX_LSH    = 6
        flann_params= dict(algorithm = FLANN_INDEX_LSH,
                            table_number = 6, # 12
                            key_size = 12,     # 20
                            multi_probe_level = 1) #2
        UA_IMG = cv2.imread('/home/hegberg/catkin_ws/src/competition_4/src/competition_4/UA_100.png')
        AR_IMG = cv2.imread('/home/hegberg/catkin_ws/src/competition_4/src/competition_4/ar.png')
        self.ua_init = False
        self.ar_init = False
        self.detector = cv2.ORB( nfeatures = 1000 )
        self.match_finder = cv2.FlannBasedMatcher(flann_params, {})  

        while not self.ua_init:
            self.ua_kp1, descrs = self.detector.detectAndCompute(UA_IMG, None)
            if descrs != None: 
                self.ua_init = True
                self.ua_d1 = descrs
                print 'UA descriptors found'     
        while not self.ar_init:
            self.ar_kp1, descrs = self.detector.detectAndCompute(AR_IMG, None)
            if descrs != None: 
                self.ar_init = True
                self.ar_d1 = descrs
                print 'AR descriptors found'   

        self.bridge = CvBridge()
        self.mode = 0 # default searching mode
        self.ua_rect = (0, 0, 360, 480)
        self.ar_rect = (0, 0, 180, 178)
        self.logo = None
        self.frame = None
        self.ar_verts = np.float32([[0, 0, 0], [0, 1, 0], [1, 1, 0], [1, 0, 0],
                               [0, 0, 1], [0, 1, 1], [1, 1, 1], [1, 0, 1], 
                               [0, 0.5, 2], [1, 0.5, 2]])
        self.house_edges = [(0, 1), (1, 2), (2, 3), (3, 0), 
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7), 
                    (4, 8), (5, 8), (6, 9), (7, 9), (8, 9)]
        # calibrated values for our turtlebot
        self.K = np.float64([[ 545.91605983,    0.        ,  306.35546651],
               [   0.        ,  542.20157959,  244.21889428],
               [   0.        ,    0.        ,    1.        ]])

        self.D = np.float64([ 0.03880064, -0.11735783,  0.00824685, -0.00252129,  0.        ])
        self.MIN_MATCH_COUNT = 10

    def detect_loop(self):
        current_frame = self.frame
        pos_msg = ''
        tag = 'NONE'
        if current_frame != None:
            kp2, d2 = self.detector.detectAndCompute(current_frame, None)
            if len(kp2) < 30 or d2 == None:
                return current_frame, pos_msg, tag

            ua_current_frame, ua_pos_msg, ua_flag, ua_length = self.match_work(kp2, d2, current_frame, pos_msg, 0)
            ar_current_frame, ar_pos_msg, ar_flag, ar_length = self.match_work(kp2, d2, current_frame, pos_msg, 1)

            if ua_flag and ua_length > ar_length:
                # found UA tag
                current_frame = ua_current_frame
                pos_msg = ua_pos_msg
                tag = 'UA'
            elif ar_flag and ar_length > 25:
                # found AR tag
                current_frame = ar_current_frame
                pos_msg = ar_pos_msg
                tag = 'AR'
        return current_frame, pos_msg, tag


    def match_work(self, kp2, d2, current_frame, pos_msg, mode):
        if mode == 0: # UA tag
            matches = self.match_finder.knnMatch(d2, self.ua_d1, k = 2)
            x0, y0, x1, y1 = self.ua_rect
            kp1 = self.ua_kp1
        else: # AR tag
            matches = self.match_finder.knnMatch(d2, self.ar_d1, k = 2)
            x0, y0, x1, y1 = (70, 70, 110, 110)
            kp1 = self.ar_kp1

        matches = [m[0] for m in matches if len(m) == 2 and m[0].distance < m[1].distance * 0.75]
        l = len(matches)
        if l < self.MIN_MATCH_COUNT:
            return current_frame, pos_msg, False, 0
        matches_by_id = [[] for _ in xrange(1)]
        for m in matches:
            matches_by_id[m.imgIdx].append(m)
        for _, matches in enumerate(matches_by_id):
            if len(matches) < self.MIN_MATCH_COUNT:
                continue
            p0 = [kp1[m.trainIdx].pt for m in matches]
            p1 = [kp2[m.queryIdx].pt for m in matches]
            p0, p1 = np.float32((p0, p1))
            H, status = cv2.findHomography(p0, p1, cv2.RANSAC, 3.0)
            status = status.ravel() != 0
            if status.sum() < self.MIN_MATCH_COUNT:
                continue
            p0, p1 = p0[status], p1[status]
            for (x, y) in np.int32(p1):
                cv2.circle(current_frame, (x, y), 2, (0, 0, 255))           
            quad = np.float32([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
            quad = cv2.perspectiveTransform(quad.reshape(1, -1, 2), H).reshape(-1, 2)
            quad3D = np.float32([[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]])
            ret, rvec, tvec = cv2.solvePnP(quad3D, quad, self.K, self.D)
            verts = self.ar_verts * [(x1-x0), (y1-y0), -(x1-x0)*0.3] + (x0, y0, 0)
            imgPts = cv2.projectPoints(verts, rvec, tvec, self.K, self.D)[0].reshape(-1, 2)
            for i in range(4):
                (x, y) = imgPts[i]
                # cv2.circle(current_frame, (int(x), int(y)), 10, (255, 0, 0), -1)
                pos_msg += str(int(x)) + ' ' + str(int(y)) + ' '
        return current_frame, pos_msg, True, l

    def img_callback(self, data):
        if self.mode == 0:
            try:
                self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print e 

    def img_callback2(self, data):
        if self.mode == 1:
            # docking, switch to front camera
            try:
                self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print e 

    def dock_callback(self, data):
        check = data.data
        if check:
            self.mode = 1
        else:
            self.mode = 0    

    def dock_callback2(self, data):
        check = data.data
        if check:
            self.mode = 0
        else:
            self.mode = 1

    def already_found_callback(self, data):
        check = data.data
        if check:
            self.mode = 0


if __name__ == '__main__':
    rospy.init_node('tag_detector')
    t = TagDetector()
    cv2.namedWindow('TAG_DETECTOR')
    img_sub = rospy.Subscriber('/cam1/camera/image_raw', Image, t.img_callback, queue_size = 1, buff_size=2**24)
    img_sub2 = rospy.Subscriber('/camera/rgb/image_raw', Image, t.img_callback2, queue_size = 1, buff_size=2**24)
    # img_sub2 = rospy.Subscriber('/cam2/image_raw', Image, t.img_callback2, queue_size = 10)
    mode_sub = rospy.Subscriber('docking_start', Bool, t.dock_callback, queue_size = 10)
    mode_sub2 = rospy.Subscriber('docking_done', Bool, t.dock_callback2, queue_size = 10)
    already_found_sub = rospy.Subscriber('already_found', Bool, t.already_found_callback, queue_size = 5)
    pub_l = rospy.Publisher('target_location', String, queue_size=5)
    pub_c = rospy.Publisher('tag_control', String, queue_size=5)
    r = rospy.Rate(10)
    while True:
        tag = 'NONE'
        current_frame, pos_msg, tag = t.detect_loop()
        if tag != 'NONE' and len(pos_msg) > 0:
            pub_l.publish(pos_msg)
            pub_c.publish(tag)
        if current_frame != None:
            cv2.imshow('TAG_DETECTOR', current_frame)
            ch = cv2.waitKey(1)
            if ch == 27:
                break   
        r.sleep()